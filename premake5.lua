workspace "physics-engine"
    architecture "x64"
    configurations { "Debug", "Release" } 

project "physics-engine"  
    kind "ConsoleApp"
    language "C++"
    targetdir "bin/%{cfg.buildcfg}/%{prj.name}"
    objdir "bin/%{cfg.buildcfg}/%{prj.name}/obj"
    files { "**.h", "**.cpp" }

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