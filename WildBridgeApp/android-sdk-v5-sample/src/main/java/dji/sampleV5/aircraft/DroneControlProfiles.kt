package dji.sampleV5.aircraft

import dji.sdk.keyvalue.key.ProductKey
import dji.sdk.keyvalue.value.product.ProductType
import dji.v5.et.create
import dji.v5.et.get
import dji.v5.manager.aircraft.payload.PayloadIndexType

enum class DroneControlProfile(
    val displayName: String,
    val maxHorizontalSpeedMps: Double,
    val maxGotoWpSpeedMps: Double,
    val distanceKp: Double,
    val distanceKi: Double,
    val distanceKd: Double,
    val yawKp: Double,
    val maxYawRateDegS: Double,
    val defaultCruiseSpeedMps: Double,
    // Payload SDK port the release/drop servo is wired to, or null if this drone
    // carries no droppable payload. Consumed by the /send/drop endpoint.
    val payloadIndexType: PayloadIndexType?
) {
    MAVIC_3_ENTERPRISE(
        displayName = "Mavic 3 Enterprise",
        maxHorizontalSpeedMps = 15.0,
        maxGotoWpSpeedMps = 5.0,
        distanceKp = 0.65,
        distanceKi = 0.0001,
        distanceKd = 0.001,
        yawKp = 3.0,
        maxYawRateDegS = 30.0,
        defaultCruiseSpeedMps = 5.0,
        payloadIndexType = PayloadIndexType.PORT_3
    ),
    MATRICE_350_RTK(
        displayName = "Matrice 350 RTK",
        maxHorizontalSpeedMps = 3.0,
        maxGotoWpSpeedMps = 3.0,
        distanceKp = 0.34,
        distanceKi = 0.0001,
        distanceKd = 0.001,
        yawKp = 3.0,
        maxYawRateDegS = 30.0,
        defaultCruiseSpeedMps = 3.0,
        payloadIndexType = PayloadIndexType.PORT_3
    ),
    MATRICE_400(
        displayName = "Matrice 400",
        maxHorizontalSpeedMps = 3.0,
        maxGotoWpSpeedMps = 3.0,
        distanceKp = 0.34,
        distanceKi = 0.0001,
        distanceKd = 0.001,
        yawKp = 3.0,
        maxYawRateDegS = 30.0,
        defaultCruiseSpeedMps = 3.0,
        payloadIndexType = PayloadIndexType.PORT_3
    ),
    MINI_4_PRO(
        displayName = "DJI Mini 4 Pro",
        maxHorizontalSpeedMps = 15.0,
        maxGotoWpSpeedMps = 5.0,
        distanceKp = 0.65,
        distanceKi = 0.0001,
        distanceKd = 0.001,
        yawKp = 3.0,
        maxYawRateDegS = 30.0,
        defaultCruiseSpeedMps = 2.0,
        payloadIndexType = null
    )
}

object DroneControlProfiles {
    fun activeProfile(): DroneControlProfile {
        val detected = ProductKey.KeyProductType.create().get(ProductType.UNKNOWN)
        return fromProductType(detected)
    }

    fun fromProductType(productType: ProductType?): DroneControlProfile {
        val name = productType?.name.orEmpty()
        return when {
            name.contains("MATRICE_400", ignoreCase = true) ||
            name.contains("M400", ignoreCase = true) -> DroneControlProfile.MATRICE_400

            name.contains("M350", ignoreCase = true) ||
            name.contains("MATRICE_350", ignoreCase = true) -> DroneControlProfile.MATRICE_350_RTK

            name.contains("MINI_4", ignoreCase = true) ||
            name.contains("MINI4", ignoreCase = true) -> DroneControlProfile.MINI_4_PRO

            name.contains("MAVIC_3", ignoreCase = true) ||
            name.contains("MAVIC3", ignoreCase = true) ||
            name.contains("M3E", ignoreCase = true) ||
            name.contains("WM265", ignoreCase = true) -> DroneControlProfile.MAVIC_3_ENTERPRISE

            else -> DroneControlProfile.MAVIC_3_ENTERPRISE
        }
    }
}