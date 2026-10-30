# Research Asset Attribution

The matrix fetches only assets named in the checked-in locks. Every downloaded
file is verified by SHA-256 before use. Fetched bytes stay below the ignored
`artifacts/rc-car-matrix/assets/` directory.

| Asset | Source and author | License | Locked content |
|---|---|---|---|
| Traffic cones | [Traffic Road Assets](https://opengameart.org/content/traffic-road-assets), MilkAndBanana | CC0-1.0 | Two pinned cone meshes; bounds and miniature scales are recorded in the asset lock. |
| Pedestrians | [Nurse](https://www.innerscene.com/tools/library/3d-parts/nurse-walking-3d-person-healthcare-cadf6540) and [Crossing Step](https://www.innerscene.com/tools/library/3d-parts/crossing-step-3d-person-walking-4235c1bb), Innerscene / MakeHuman | CC0-1.0 | Two pinned human meshes, scaled to miniature scene dimensions. |
| Road vehicles | [Compact SUV](https://www.innerscene.com/tools/library/3d-parts/generic-compact-suv-9f157ee5) and [Convertible](https://www.innerscene.com/tools/library/3d-parts/generic-convertible-roadster-05aace60), Innerscene | CC0-1.0 | Two pinned vehicle meshes, scaled to the RC course. |
| Road surfaces | [Asphalt](https://ambientcg.com/view?id=Asphalt001), [Paving Stones](https://ambientcg.com/view?id=PavingStones001), and [Concrete](https://ambientcg.com/view?id=Concrete017), ambientCG | CC0-1.0 | Three pinned material archives; 1 m texture tile is a labelled placeholder. |
| Controlled vehicle visual | [Pickup truck](https://www.innerscene.com/tools/library/3d-parts/pickup-truck-da45aa3f), Innerscene | CC0-1.0 | Pinned GLB at 0.08 visual scale; visual only. Physics, collisions, sensors, and actuation remain in the RC SDF. |

The complete source revision, archive/member hash, file hash, dimensions,
scale, and provenance are stored in
`ros2_ws/src/car_bringup/config/rc_module_matrix/research_assets.lock.yaml`
and `research_vehicle.lock.yaml`. The source terms and CC0 grant were reviewed
at the linked pages. These locked source meshes are the assets used by the
matrix worlds.
