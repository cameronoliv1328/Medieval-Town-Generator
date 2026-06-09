# Implemented Medieval Morphology Rules

This file captures the concrete rule translation used by the C++ PCG nodes.

## Historical logic (macro)
- Anchors generated for: center, market, keep, church, and gates.
- Gate count and wall radius controlled by archetype settings.
- Town reasons supported by enum for profile-driven presets.

## Street hierarchy and growth
- Primary stage: gate/bridge trunks routed by terrain-aware A* into an
  irregular market-square ring; later trunks merge into earlier ones
  (Y-junctions) instead of all meeting at one point.
- Secondary/lane/alley stages: block growth — streets sprout from existing
  frontages and grow until they strike another street (T-junction), snap to a
  node, or remain as dead-end lanes. Blocks emerge as closed faces.
- Street tiers include primary/secondary/lane/alley (+ quay river paths).
- Importance metric emitted per segment for downstream readability.

## Parcels (implemented in OrganicParcelGenerator)
- Every street frontage subdivides into burgage plots: narrow frontage
  (FrontageMin..Max), depth several times the frontage, scaled by street tier
  and local urbanity.
- Occupancy grid arbitrates space; facing streets split tight corridors
  fairly; the market ring claims frontage first.
- A relaxed infill pass adds hovels/encroachments to leftover frontage.
- Parcel metadata stores plot polygon, frontage/depth, wealth, party walls,
  corner/riverfront flags.

## Buildings
- District + wealth + mixed-use drives footprint type assignment.
- Core districts prefer shopfront/rowhouse and party walls.
- Noble parcels can become courtyard houses.
- Poor parcels bias toward huts.
- Rear outbuildings emitted when parcel depth supports yards.

## Districting
- Continuous fields, no hard rings/wedges: wealth = market + landmark
  proximity + noise - river-industry penalty; urbanity = market pull + noise
  + wall fade.
- River band biases kinds toward warehouses/smithies (working quays).
- District enum for PCG attributes derived per-lot from the fields.
