workspace "physics-engine"
    architecture "x64"
    configurations { "Debug", "Release" } 

project "physics-engine"
    location "./"
    kind "ConsoleApp"
    language "C++"
    targetdir "%{prj.name}/bin/%{cfg.buildcfg}/"
    objdir "%{prj.name}/bin/%{cfg.buildcfg}/obj"
    includedirs { "%{prj.name}/include" }
    files { "%{prj.name}/src/**.cpp", "%{prj.name}/src/**.h" }

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