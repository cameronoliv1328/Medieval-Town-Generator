using UnrealBuildTool;

public class MedievalTownGeneratorPCG : ModuleRules
{
    public MedievalTownGeneratorPCG(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "PCG",
            "PCGGraph",
            "GeometryCore",
            "GeometryFramework"
        });

        PrivateDependencyModuleNames.AddRange(new[]
        {
            "RenderCore",
            "RHI",
            "Landscape",
            "DeveloperSettings"
        });
    }
}
