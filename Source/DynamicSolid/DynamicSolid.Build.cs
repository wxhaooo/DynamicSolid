// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class DynamicSolid : ModuleRules
{
	public DynamicSolid(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[]
		{ "Core", "CoreUObject", "Engine","InputCore",
			"RHI", "RenderCore","RuntimeMeshComponent" });

		PrivateDependencyModuleNames.AddRange(new string[] {  });

		LoadThirdPartyLibrary();

        // Uncomment if you are using Slate UI
        PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

		if(Target.bBuildEditor)
        {
			PrivateDependencyModuleNames.AddRange(new string[] { "DetailCustomizations", "PropertyEditor","EditorStyle" });
			PublicDependencyModuleNames.AddRange(new string[] { "UnrealEd" });
		}
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}

	public void LoadThirdPartyLibrary()
	{
		//load ThirdParty Header
		PublicIncludePaths.Add("../ThirdParty/include");

		//load ThirdParty Libs
		//PublicAdditionalLibraries.Add("../ThirdParty/libs/AutodeskFBX");
	}
}
