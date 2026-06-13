package dji.sampleV5.aircraft.controller

import android.os.Handler
import android.os.Looper
import android.util.Log
import dji.sampleV5.aircraft.models.MediaVM
import dji.sampleV5.aircraft.models.PayloadWidgetVM
import dji.sampleV5.aircraft.util.ToastUtils
import dji.sdk.keyvalue.key.CameraKey
import dji.sdk.keyvalue.key.DJIKey
import dji.sdk.keyvalue.key.KeyTools
import dji.sdk.keyvalue.value.camera.GeneratedMediaFileInfo
import dji.sdk.keyvalue.value.camera.LaserMeasureInformation
import dji.sdk.keyvalue.value.camera.LaserMeasureState
import dji.sdk.keyvalue.value.camera.LaserWorkMode
import dji.v5.common.callback.CommonCallbacks
import dji.v5.common.error.IDJIError
import dji.v5.et.create
import dji.v5.et.get
import dji.v5.et.listen
import dji.v5.et.set
import dji.v5.manager.KeyManager
import dji.v5.manager.aircraft.payload.PayloadIndexType
import dji.v5.manager.datacenter.media.MediaFile
import dji.v5.manager.datacenter.media.MediaFileDownloadListener
import dji.sdk.keyvalue.value.payload.WidgetType
import dji.sdk.keyvalue.value.payload.WidgetValue
import java.io.OutputStream
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit

/**
 * H20T payload control — Laser Range Finder (LRF) and thermal photo capture.
 *
 * Singleton so the HTTP endpoints in WildBridgeDefaultLayoutActivity can fire payload
 * actions without holding payload-specific state in the activity. The blocking calls
 * (takeFreshLrfReading, captureThermal, sendCapturedImage) are invoked from the HTTP
 * server's worker threads, and are expected to run one at a time — the command server
 * issues capture requests serially.
 */
object Payload {

    private const val TAG = "Payload"

    // ==================== Laser Range Finder (LRF) ====================

    private val laserKey: DJIKey<LaserWorkMode> = CameraKey.KeyLaserWorkMode.create()

    // Master on/off for the rangefinder — the same toggle DJI Pilot 2 exposes. KeyLaserWorkMode
    // only selects the firing mode (on-demand vs. always); measurement stays OFF (every
    // LaserMeasureInformation field null) until this is set true. Enabling it here removes the need
    // to flip the laser on manually in DJI Pilot 2.
    private val laserEnabledKey: DJIKey<Boolean> = CameraKey.KeyLaserMeasureEnabled.create()

    // Enable the rangefinder and set it to fire continuously. Idempotent; safe to call repeatedly.
    private fun enableLaser() {
        laserEnabledKey.set(
            true,
            onSuccess = { Log.i(TAG, "LRF measure enabled") },
            onFailure = { error -> Log.e(TAG, "LRF measure enable failed: ${error.description()}") }
        )
        laserKey.set(
            LaserWorkMode.OPEN_ALWAYS,
            onSuccess = { Log.i(TAG, "LRF laser opened") },
            onFailure = { error -> Log.e(TAG, "LRF laser open failed: ${error.description()}") }
        )
    }

    init { enableLaser() }

    private val laserMeasureKey: DJIKey<LaserMeasureInformation> = CameraKey.KeyLaserMeasureInformation.create()
    private val lrfReadingLock = Any()

    @Volatile
    private var lrfInfo: LaserMeasureInformation? = null
    @Volatile
    private var lrfListenerRegistered: Boolean = false


    // Register a persistent listener that caches the latest laser measurement. Idempotent.
    private fun setupLaserMeasureListener() {
        if (lrfListenerRegistered) return
        // Cache the latest measurement as a fallback; takeFreshLrfReading polls the key directly.
        laserMeasureKey.listen(this) { newValue: LaserMeasureInformation? ->
            lrfInfo = newValue
        }
        lrfListenerRegistered = true
        Log.i(TAG, "LRF measure listener registered")
    }

    // Fire the laser, wait for a fresh measurement, then turn the laser off.
    // Blocking, call from a worker thread.
    fun takeFreshLrfReading(timeoutMs: Long = 2000L): LaserMeasureInformation? = synchronized(lrfReadingLock) {
        setupLaserMeasureListener()
        lrfInfo = null

        try {
            // Re-apply: the init-time enable runs before the SDK is connected and silently fails,
            // so the first real reading must turn the laser on itself.
            enableLaser()

            // Poll the key directly for fresh values; wait for the laser to lock (state == NORMAL)
            val deadline = System.currentTimeMillis() + timeoutMs
            var reading: LaserMeasureInformation? = null
            while (System.currentTimeMillis() < deadline) {
                val current = laserMeasureKey.get() ?: lrfInfo
                if (current != null) {
                    reading = current
                    if (current.laserMeasureState == LaserMeasureState.NORMAL) break
                }
                try {
                    Thread.sleep(50)
                } catch (e: InterruptedException) {
                    Thread.currentThread().interrupt()
                    break
                }
            }
            if (reading == null) {
                Log.w(TAG, "LRF reading timed out after ${timeoutMs}ms (no data from laser)")
            } else {
                Log.i(TAG, "LRF reading: ${reading.distance} m, state=${reading.laserMeasureState}")
            }
            reading
        } finally {
//            // Return the laser to on-demand (closed)
//            laserKey.set(
//                LaserWorkMode.OPEN_ON_DEMAND,
//                onSuccess = { Log.i(TAG, "LRF laser set to OPEN_ON_DEMAND (off)") },
//                onFailure = { error -> Log.e(TAG, "LRF laser close failed: ${error.description()}") } )
        }
    }
    // ==================== Thermal photo capture (H20T) ====================

    private val mainHandler = Handler(Looper.getMainLooper())

    // Fires when the camera writes a new photo to the SD card; carries the new file's index.
    private val keyNewlyGeneratedMediaFile = KeyTools.createKey(CameraKey.KeyNewlyGeneratedMediaFile)
    @Volatile
    private var latestGeneratedMediaInfo: GeneratedMediaFileInfo? = null
    @Volatile
    private var newMediaListenerRegistered = false

    // Lens files from each shutter, kept so they can be downloaded individually on a
    // follow-up request. Keyed by the captureId handed back to the caller.
    private val captures = java.util.concurrent.ConcurrentHashMap<String, ThermalCapture>()

    // Persistent listener caching the most-recent generated-file info. Registered once
    // (idempotent) and torn down via destroy(). A fresh listener per capture would replay the
    // PREVIOUS photo's cached value on registration, which is what made repeated captures return
    // the same stale file.
    private fun setupNewMediaListener() {
        if (newMediaListenerRegistered) return
        keyNewlyGeneratedMediaFile.listen(this) { newValue: GeneratedMediaFileInfo? ->
            latestGeneratedMediaInfo = newValue
        }
        newMediaListenerRegistered = true
    }

    // A single H20T shutter writes up to three co-aligned files to the SD card with the same
    // timestamp and scene: a radiometric thermal R-JPEG, the wide visual (RGB) photo, and the
    // zoom-lens photo. These are the hardware-synchronized counterparts to the thermal frame —
    // strictly better aligned than tapping the (separately-pipelined) WHIP live stream. Each is
    // null when the H20T is not configured to store that lens.
    data class ThermalCapture(val thermal: MediaFile?, val visual: MediaFile?, val zoom: MediaFile?)

    // H20T per-lens filename suffixes (before the extension): _T = thermal R-JPEG,
    // _W = wide / _V = visible RGB, _Z = zoom.
    private fun baseNameNoExt(name: String?): String =
        (name ?: "").substringBeforeLast('.').uppercase()

    private fun isThermalName(name: String?): Boolean = baseNameNoExt(name).endsWith("_T")

    private fun isWideName(name: String?): Boolean =
        baseNameNoExt(name).let { it.endsWith("_W") || it.endsWith("_V") }

    private fun isZoomName(name: String?): Boolean = baseNameNoExt(name).endsWith("_Z")

    // Fire one shutter and return the thermal, wide-visual (RGB) and zoom MediaFiles from it.
    // Blocking, call from a worker thread.
    fun takeThermalAndVisual(mediaVM: MediaVM): ThermalCapture {
        val newFiles = captureNewMediaFiles(mediaVM)
        if (newFiles.isEmpty()) return ThermalCapture(null, null, null)

        // The H20T may surface the lens images of one shot either as separate sibling entries
        // or grouped under a parent MediaFile (subMediaFile). Flatten both so classification
        // sees every variant.
        val variants = LinkedHashMap<String, MediaFile>()
        fun add(file: MediaFile?) {
            if (file?.fileName == null) return
            variants.putIfAbsent(file.fileName, file)
            file.subMediaFile?.forEach { sub -> sub?.fileName?.let { variants.putIfAbsent(it, sub) } }
        }
        newFiles.forEach { add(it) }
        val all = variants.values.toList()
        Log.i(TAG, "Lens files this shutter: ${all.map { "${it.fileName}#${it.fileIndex}(${it.fileSize}B)" }}")

        // Thermal: the _T file by name; failing that the SMALLEST file (the radiometric R-JPEG is
        // far smaller than the full-res wide/zoom visuals), which is a much safer fallback than
        // blindly trusting the first-reported file.
        val thermal = all.firstOrNull { isThermalName(it.fileName) }
            ?: all.minByOrNull { it.fileSize }
            ?: newFiles.first()
        // Wide visual (RGB) and zoom siblings, distinguished by lens suffix.
        val visual = all.firstOrNull { isWideName(it.fileName) && it.fileName != thermal.fileName }
        val zoom = all.firstOrNull { isZoomName(it.fileName) && it.fileName != thermal.fileName }

        if (visual == null && zoom == null) {
            Log.w(TAG, "No wide/zoom siblings found — H20T may be set to store infrared only")
        } else {
            Log.i(TAG, "Paired thermal=${thermal.fileName} (${thermal.fileSize}B) " +
                "visual=${visual?.fileName} (${visual?.fileSize}B) zoom=${zoom?.fileName} (${zoom?.fileSize}B)")
        }
        return ThermalCapture(thermal, visual, zoom)
    }

    private fun jsonName(name: String?) = if (name == null) "null" else "\"$name\""

    // Fire one shutter (thermal + RGB + zoom, hardware-synchronized), remember the lens files
    // for later per-lens download, and return a JSON descriptor naming the captureId and which
    // lenses the H20T actually stored (null where absent). Returns null if the shutter produced
    // no thermal file. Blocking, call from a worker thread.
    fun captureThermal(mediaVM: MediaVM): String? {
        val capture = takeThermalAndVisual(mediaVM)
        if (capture.thermal == null) return null
        val captureId = System.currentTimeMillis().toString()
        captures[captureId] = capture
        // Bound memory: keep only the most recent few captures.
        if (captures.size > 8) {
            captures.keys.sorted().take(captures.size - 8).forEach { captures.remove(it) }
        }
        return "{\"captureId\":\"$captureId\"," +
            "\"thermal\":${jsonName(capture.thermal.fileName)}," +
            "\"wide\":${jsonName(capture.visual?.fileName)}," +
            "\"zoom\":${jsonName(capture.zoom?.fileName)}}"
    }

    // Trip one shutter on the H20T and return ALL media files it produced (thermal R-JPEG plus,
    // when visible storage is enabled, the wide/zoom visual photo). Blocking, call from a worker
    // thread. mediaVM must be init'd with SD card storage and the LEFT_OR_MAIN component index
    // (done in the host activity's onCreate).
    private fun captureNewMediaFiles(mediaVM: MediaVM): List<MediaFile> {
        try {
            setupNewMediaListener()

            // Remember the current newest-photo index (from the key cache or the last
            // listener push) so the shot we are about to take can be told apart from it.
            val baselineIndex = (keyNewlyGeneratedMediaFile.get() ?: latestGeneratedMediaInfo)?.index
            latestGeneratedMediaInfo = null

            // Trip the shutter on the main thread and wait for the SDK callback.
            var photoError: String? = null
            val photoLatch = CountDownLatch(1)
            mainHandler.post {
                mediaVM.takePhoto(object : CommonCallbacks.CompletionCallback {
                    override fun onSuccess() {
                        photoLatch.countDown()
                    }
                    override fun onFailure(error: IDJIError) {
                        photoError = error.description()
                        Log.e(TAG, "Photo capture failed: $photoError")
                        photoLatch.countDown()
                    }
                })
            }
            if (!photoLatch.await(8, TimeUnit.SECONDS)) {
                Log.e(TAG, "Timeout waiting for shutter")
                return emptyList()
            }
            if (photoError != null) return emptyList()

            // Wait for a new index to appear from either the listener push or the key cache
            var newIndex: Int? = null
            val idxDeadline = System.currentTimeMillis() + 8000L
            while (System.currentTimeMillis() < idxDeadline) {
                // Accept a fresh index from EITHER the listener push or the key cache,
                // whichever first reports something different from the baseline
                val cur = listOfNotNull(
                    latestGeneratedMediaInfo?.index,
                    keyNewlyGeneratedMediaFile.get()?.index
                ).firstOrNull { it != baselineIndex }
                if (cur != null) {
                    newIndex = cur
                    break
                }
                Thread.sleep(100)
            }
            if (newIndex == null) {
                Log.e(TAG, "No new media index after shutter (baseline=$baselineIndex)")
                return emptyList()
            }
            Log.i(TAG, "New photo index=$newIndex (baseline=$baselineIndex)")

            // Pull a WINDOW around the new index, not a single entry: one H20T shutter writes
            // several lens files (thermal + wide/zoom visual) at adjacent indices, and the
            // "newly generated" key reports only one of them. A ±window pull surfaces them all.
            val window = 6
            val start = (newIndex - window).coerceAtLeast(0)
            mainHandler.post { mediaVM.pullMediaFileListFromCamera(start, window * 2 + 1) }

            val listDeadline = System.currentTimeMillis() + 10000L
            while (System.currentTimeMillis() < listDeadline) {
                val data = mediaVM.mediaFileListData.value?.data
                if (data?.any { it.fileIndex == newIndex } == true) {
                    // Files from THIS shutter are exactly those newer than the pre-shutter baseline
                    // (captures are serial). With no baseline (first shot), fall back to the window.
                    val newFiles = data.filter {
                        if (baselineIndex != null) it.fileIndex > baselineIndex
                        else it.fileIndex in (newIndex - window)..(newIndex + window)
                    }
                    Log.i(TAG, "Resolved ${newFiles.size} new file(s): ${newFiles.map { "${it.fileName}#${it.fileIndex}" }}")
                    return newFiles
                }
                Thread.sleep(100)
            }
            Log.e(TAG, "File list never surfaced index $newIndex")
            return emptyList()
        } catch (e: Exception) {
            Log.e(TAG, "Error taking thermal image: ${e.message}", e)
            return emptyList()
        }
        // Note: takePhoto switches the camera to PHOTO_NORMAL and we intentionally leave it
        // there. Restoring VIDEO_NORMAL after each shot made the controller visibly flip out
        // of image-capture mode ("blinking") between captures.
    }

    // Download a MediaFile from the camera straight into memory (no disk round-trip) and return
    // its bytes, or null on failure. Blocking. The DJI-link download is the bottleneck; skipping
    // the write-to-flash-then-read-back saves tens of MB of needless I/O per capture.
    private fun downloadToBytes(photoFile: MediaFile): ByteArray? {
        Log.i(TAG, "Downloading ${photoFile.fileName} (${photoFile.fileSize} bytes) to memory")
        // Pre-size the buffer to the known file size to avoid repeated array growth/copies.
        val initialCapacity = photoFile.fileSize.takeIf { it in 1..Int.MAX_VALUE.toLong() }?.toInt() ?: (1 shl 20)
        val buffer = java.io.ByteArrayOutputStream(initialCapacity)

        val downloadLatch = CountDownLatch(1)
        var downloadError: String? = null

        photoFile.pullOriginalMediaFileFromCamera(0L, object : MediaFileDownloadListener {
            override fun onStart() { /* no-op */ }
            override fun onProgress(total: Long, current: Long) { /* no-op */ }
            override fun onRealtimeDataUpdate(data: ByteArray, position: Long) {
                // Callbacks arrive in order from offset 0, so a plain append reconstructs the file.
                buffer.write(data, 0, data.size)
            }
            override fun onFinish() {
                downloadLatch.countDown()
            }
            override fun onFailure(error: IDJIError?) {
                downloadError = error?.description() ?: "Unknown download error"
                Log.e(TAG, "Download failed for ${photoFile.fileName}: $downloadError")
                downloadLatch.countDown()
            }
        })

        if (!downloadLatch.await(120, TimeUnit.SECONDS)) {
            Log.e(TAG, "Timeout downloading ${photoFile.fileName}")
            return null
        }
        if (downloadError != null) return null
        val bytes = buffer.toByteArray()
        if (bytes.isEmpty()) {
            Log.e(TAG, "Downloaded ${photoFile.fileName} is empty")
            return null
        }
        Log.i(TAG, "Downloaded ${photoFile.fileName} (${bytes.size} bytes) to memory")
        return bytes
    }

    // Look up a previously-captured lens file (by captureId) and stream it back as a single
    // image/jpeg HTTP response. [lens] is "thermal", "wide" or "zoom". Writes an error
    // response if the captureId is unknown, the requested lens wasn't stored for that shutter, or
    // the download fails. Blocking, call from a worker thread.
    fun sendCapturedImage(captureId: String, lens: String, outputStream: OutputStream) {
        val capture = captures[captureId]
        if (capture == null) {
            sendErrorResponse(outputStream, "Unknown captureId: $captureId")
            return
        }
        val mediaFile = when (lens.lowercase()) {
            "thermal" -> capture.thermal
            "wide" -> capture.visual
            "zoom" -> capture.zoom
            else -> null
        }
        if (mediaFile == null) {
            sendErrorResponse(outputStream, "Lens '$lens' not available for capture $captureId")
            return
        }

        try {
            val bytes = downloadToBytes(mediaFile)
            if (bytes == null) {
                sendErrorResponse(outputStream, "Failed to download $lens image")
                return
            }
            val headers = StringBuilder().apply {
                append("HTTP/1.1 200 OK\r\n")
                append("Content-Type: image/jpeg\r\n")
                append("Content-Length: ${bytes.size}\r\n")
                append("Content-Disposition: attachment; filename=\"${mediaFile.fileName}\"\r\n")
                append("Access-Control-Allow-Origin: *\r\n")
                append("\r\n")
            }
            outputStream.write(headers.toString().toByteArray())
            outputStream.write(bytes)
            outputStream.flush()

            Log.i(TAG, "Sent $lens image ${mediaFile.fileName} (${bytes.size} bytes)")
            mainHandler.post {
                ToastUtils.showToast("Sent $lens image (${bytes.size / 1024} KB)")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error sending $lens image: ${e.message}", e)
            try {
                sendErrorResponse(outputStream, "Error sending $lens image: ${e.message}")
            } catch (responseError: Exception) {
                Log.e(TAG, "Failed to send error response: ${responseError.message}")
            }
        }
    }

    fun sendErrorResponse(outputStream: OutputStream, message: String) {
        val body = message.toByteArray()
        val headers = StringBuilder().apply {
            append("HTTP/1.1 500 Internal Server Error\r\n")
            append("Content-Type: text/plain\r\n")
            append("Content-Length: ${body.size}\r\n")
            append("Access-Control-Allow-Origin: *\r\n")
            append("\r\n")
        }
        outputStream.write(headers.toString().toByteArray())
        outputStream.write(body)
        outputStream.flush()
    }

    // ==================== Payload drop (release servo) ====================

    // The PayloadIndexType the widget VM is currently wired to. initListener() must run once
    // before setWidgetValue() (it sets a lateinit field in the VM); we re-register only when
    // the port changes so repeated drops don't stack duplicate payload listeners.
    @Volatile
    private var dropListenerIndex: PayloadIndexType? = null

    // Fire the payload release on [indexType], reproducing the manual main-interface widget
    // sequence: arm the unlock SWITCH, pulse the release BUTTON on then off, then re-lock.
    // The 300 ms gaps let the payload firmware register each discrete widget change.
    //
    // [armSwitchIndex]/[releaseButtonIndex] are payload widgetIndex values supplied by the
    // active DroneControlProfile. The original working PORT_3 logic (M400/M3E/M350) is
    // SWITCH 0 + BUTTON 1; the M300 SkyPort payload (TH4) uses Unlock SWITCH 3 + All_Down
    // BUTTON 5.
    //
    // Blocking, call from a worker thread. payloadWidgetVM must be obtained from the host
    // activity via ViewModelProvider (created on the main thread).
    fun dropPayload(
        payloadWidgetVM: PayloadWidgetVM,
        indexType: PayloadIndexType,
        armSwitchIndex: Int = 0,
        releaseButtonIndex: Int = 2
    ): Boolean {
        return try {
            if (dropListenerIndex != indexType) {
                payloadWidgetVM.initListener(indexType)
                dropListenerIndex = indexType
            }

            val armSwitch = WidgetValue().apply {
                type = WidgetType.SWITCH
                index = armSwitchIndex
                value = 1
            }
            payloadWidgetVM.setWidgetValue(armSwitch)
            Thread.sleep(300)

            val releaseButton = WidgetValue().apply {
                type = WidgetType.BUTTON
                index = releaseButtonIndex
                value = 1
            }
            payloadWidgetVM.setWidgetValue(releaseButton)
            Thread.sleep(300)

            releaseButton.value = 0
            payloadWidgetVM.setWidgetValue(releaseButton)
            Thread.sleep(300)

            armSwitch.value = 0
            payloadWidgetVM.setWidgetValue(armSwitch)

            Log.i(TAG, "Payload drop sequence sent on $indexType (switch=$armSwitchIndex, button=$releaseButtonIndex)")
            true
        } catch (e: Exception) {
            Log.e(TAG, "Payload drop failed: ${e.message}", e)
            false
        }
    }

    // Cancel payload key listeners. Call from the host activity's onDestroy.
    fun destroy() {
        KeyManager.getInstance().cancelListen(this)
        lrfListenerRegistered = false
        lrfInfo = null
        newMediaListenerRegistered = false
        latestGeneratedMediaInfo = null
        dropListenerIndex = null
    }
}
