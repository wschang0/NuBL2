

    AREA _NuBL3xFwInfo, DATA, READONLY

    EXPORT  g_NuBL32InfoStart
    EXPORT  g_NuBL32InfoEnd
    EXPORT  g_NuBL33InfoStart
    EXPORT  g_NuBL33InfoEnd

    ALIGN   4
        
g_NuBL32InfoStart
    INCBIN ..\keil\tfm_s_info.bin
g_NuBL32InfoEnd


g_NuBL33InfoStart
    INCBIN ..\keil\tfm_ns_info.bin
g_NuBL33InfoEnd

    END
