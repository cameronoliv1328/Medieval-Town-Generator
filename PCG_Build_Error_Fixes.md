# Fixing "Unable to find parent class UPCGSettings" in UE5

If UE reports errors like:
- `Unable to find parent class for 'UMedievalCityMacroSettings' named 'UPCGSettings'`

then your project target is compiling without the PCG module/plugin available to UHT.

## Required project-side steps
1. Enable the **PCG** plugin in your `.uproject` (Edit → Plugins → Procedural Content Generation).
2. Add `PCG` to your **actual game module** `<YourModule>.Build.cs` dependencies (not just this sample file).
3. Regenerate Visual Studio project files.
4. Do a full rebuild.

## Safety fallback included in this repo
The new settings headers now compile even when PCG is absent by using a compatibility base type:
- PCG available → derives from `UPCGSettings`
- PCG unavailable → derives from `UObject`

This prevents hard UHT parent-class failures and lets non-PCG targets compile.

## Important
When PCG is unavailable, custom PCG node execution code is intentionally disabled at compile-time.
