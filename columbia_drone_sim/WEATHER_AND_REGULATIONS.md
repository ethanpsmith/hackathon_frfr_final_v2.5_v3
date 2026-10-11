# Columbia weather provenance and operational review

Research date: 10 October 2026. Recheck rules and site conditions before real operations. This is an engineering summary, not legal advice or an approval determination.

## Weather basis

The offline dataset combines NWS Columbia 1991–2020 temperature/precipitation normals [1] with measured annual wind/humidity values from the 2024 KCAE annual climate report issued 1 January 2025 [2]. The report-year identification comes from the opened report body; search snippets and rolling report URLs may describe another version.

| Variable | Reference | Simulation use |
|---|---|---|
| October mean temperature | 64.6°F / 18.11°C, 1991–2020 [1] | Default temperature |
| Annual mean temperature | 64.3°F / 17.94°C, 1991–2020 [1] | Background context |
| October precipitation | 3.13 in / 79.50 mm per month [1] | Context, not a constant rain rate |
| Annual precipitation | 45.24 in / 1,149.10 mm [1] | Context |
| Mean wind | 5.8 mph / 2.593 m/s, calendar 2024 [2] | Constructed mean surface-wind scenario |
| Mean relative humidity | 70%, calendar 2024 [2] | Moist-air density scenario |
| Highest reported gust | 67 mph / 29.95 m/s, calendar 2024 [2] | Shows why an average is not an operational limit |

The full monthly normal table is in `data/columbia_climate.json`. A 240° wind-from direction is an editable scenario choice; the annual resultant is not a prevailing-direction probability distribution. No synthetic field is described as a measured climatological normal.

Pressure, cloud separation, dry weather, visibility, daylight, turbulence and shear exponent in offline mode are assumptions. Limits of 8 m/s mean wind at aircraft height, 12 m/s instantaneous wind, 5–38°C ambient and 55°C pack temperature are proposed project limits, not established capabilities of this STL or FAA universal limits.

The optional NWS API [3] supplies station observations and point alerts. An observation is rejected if over 120 minutes old, implausibly future-dated, or missing required values. It cannot establish runway/route/cloud/landing-area conditions. Horizontal cloud clearance remains an assumption. Weather observations initialize a snapshot; they do not become a continuous live feed. Approximate solar elevation [4] is used for a conservative simulated daylight gate, not legal sunrise/sunset certification.

For deployment, obtain route-level forecasts/alerts, local hub and destination measurements, mission-duration freshness requirements and operator-approved weather minima. Airport means do not capture buildings, tree lines, gust fronts or thunderstorms. Do not use averages to authorize launches.

## Rules represented numerically

The reference Part 107 screens use a 400 ft / 121.92 m AGL ceiling, 100 mph / 44.704 m/s groundspeed ceiling, at least 3 statute miles / 4,828.032 m visibility, 500 ft / 152.4 m below cloud and 2,000 ft / 609.6 m horizontal cloud separation [5]. This project does not implement the structure-related altitude exception. Its default 60 m altitude and 12 m/s speed are narrower project choices. Dry daylight operations are a project restriction, not a statement that every other operation is universally prohibited.

The package-transport Part 107 pathway requires applicable conditions including VLOS, secure loads, less than 55 lb total takeoff mass and the relevant intrastate restrictions [6]. A numerical distance is not a legal definition of VLOS. A several-mile delivery must not be assumed to satisfy it.

## What is not authorized or verified

FAA guidance describes commercial BVLOS package-delivery operations through Part 135 and applicable aircraft/operator/airspace approvals [7]. An autonomous mission still needs the authorized operational accountability and supervision specified in those approvals. A remote pilot is not an onboard pilot. The FAA BVLOS page describes a proposed rule [8]; this package does not treat proposed Part 108 as effective permission.

Remote ID compliance is separate from airspace/operating authority. Standard Remote ID, module and FRIA pathways differ; a broadcast module's VLOS restriction is especially relevant [9]. This simulator broadcasts nothing.

Columbia route selection must consider nearby civil/military airports, controlled or special-use airspace, temporary restrictions and stadium/event restrictions using current official information [10]. Merely being below 400 ft or on private property is insufficient. This program does not identify restricted polygons or claim a safe corridor. The displayed rectangle is only a **virtual test boundary**.

Before real flights, a qualified operator must resolve, document and periodically reverify:

1. Applicable aircraft and operator certifications, exemptions, waivers and operations specifications.
2. Registration and Remote ID, qualified personnel, supervision limits and maintenance records.
3. Exact airspace/TFR/NOTAM/authorization status and any required ATC coordination.
4. Surveyed route, ground risk, operations over people/moving vehicles, detect-and-avoid and emergency landing areas.
5. Property/landing permission, zoning, privacy, noise, insurance and applicable state/local rules.
6. Weather minima, forecasts, communication/navigation contingencies and energy reserves.
7. Cargo security, exclusion of hazardous materials, validated release interlocks and public protection.
8. Any applicable environmental review and community-engagement requirements.

No approval has been supplied for the reference warehouse or Site A. All review items start unverified. Changing a JSON checkbox is not evidence of approval; the application never changes its real-flight status to authorized. Its purpose is to develop and test an operating concept before the operational review.

## Sources

[1] NWS Columbia, *New Climate Normals*, Columbia tables on PDF pages 11 and 13. https://www.weather.gov/media/cae/New_Climate_Normals_public.pdf

[2] NWS Columbia, *Columbia Metro Airport climate summary for 2024*, issued 1 January 2025, 08:50 EST. Rolling product URL accessed for the stated report body: https://forecast.weather.gov/product.php?format=CI&glossary=1&issuedby=CAE&product=CLA&site=ARX&version=1

[3] NWS API documentation. https://www.weather.gov/documentation/services-web-api

[4] NOAA, *General Solar Position Calculations*. https://gml.noaa.gov/grad/solcalc/solareqns.PDF

[5] eCFR, 14 CFR 107.51. https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-107/subpart-B/section-107.51

[6] FAA, Package delivery under Part 107. https://www.faa.gov/faq/package-delivery-allowed-under-small-uas-rule-part-107

[7] FAA, Package delivery by drone / Part 135. https://www.faa.gov/uas/advanced_operations/package_delivery_drone

[8] FAA, Beyond Visual Line of Sight. https://www.faa.gov/newsroom/beyond-visual-line-sight-bvlos

[9] FAA, Remote Identification of Drones. https://www.faa.gov/uas/getting_started/remote_id

[10] FAA, Airspace Restrictions. https://www.faa.gov/uas/getting_started/where_can_i_fly/airspace_restrictions

[11] Matplotlib animation API (desktop visualization implementation). https://matplotlib.org/stable/api/animation_api.html
