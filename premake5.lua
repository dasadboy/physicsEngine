workspace "physics-engine"
    architecture "x64"
    configurations { "Debug", "Release" } 

project "physics-engine"
    location "./"
    kind "ConsoleApp"
    language "C++"
    targetdir "bin/%{cfg.buildcfg}/%{prj.name}"
    objdir "bin/%{cfg.buildcfg}/%{prj.name}/obj"
    files { "./src/**.cpp", "./src/**.h" }
    includedirs { "./include" }

    filter "system:windows"
        cppdialect "C++20"
        staticruntime "On"
        systemversion "latest"

    filter "configurations:Debug"
        defines { "DEBUG" } 
        symbols "On"

    filter "configurations:Release"
        defines { "NDEBUG" }
        optimize "On"