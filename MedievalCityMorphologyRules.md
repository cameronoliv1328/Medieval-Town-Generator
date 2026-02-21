# Implemented Medieval Morphology Rules

This file captures the concrete rule translation used by the C++ PCG nodes.

## Historical logic (macro)
- Anchors generated for: center, market, keep, church, and gates.
- Gate count and wall radius controlled by archetype settings.
- Town reasons supported by enum for profile-driven presets.

## Street hierarchy and growth
- Primary stage: gate-to-market spines.
- Secondary stage: sampled attractors connect organically under minimum intersection spacing.
- Street tiers include primary/secondary/alley/lane/wall-road.
- Importance metric emitted per segment for downstream readability.

## Parcels
- Core parcels use burgage-like frontage/depth ranges.
- Back-lane flag generated when depth exceeds threshold and probability check passes.
- Non-core parcels jitter for irregular medieval edge behavior.
- Parcel metadata stores district/frontage/depth/wealth/mixed-use/access proxies.

## Buildings
- District + wealth + mixed-use drives footprint type assignment.
- Core districts prefer shopfront/rowhouse and party walls.
- Noble parcels can become courtyard houses.
- Poor parcels bias toward huts.
- Rear outbuildings emitted when parcel depth supports yards.

## Districting
- Weighted radial + noise partition.
- River-side logic separates crafts/docks tendency.
- Keep and church proximity create noble/church wards.
