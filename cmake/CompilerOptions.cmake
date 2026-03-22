function(adsim_set_compile_options target)
    target_compile_options(${target} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang>:
            -Wall
            -Wextra
            -Wpedantic
            -Wconversion
            -Wshadow
            -Wnon-virtual-dtor
            -Woverloaded-virtual
            -Wno-unused-parameter
        >
        $<$<CXX_COMPILER_ID:MSVC>:
            /W4
            /permissive-
            /wd4100
        >
    )
    target_compile_features(${target} PRIVATE cxx_std_20)
endfunction()
