idf_component_register(SRCS UI/screens/ui_Screen2.c
    UI/screens/ui_Screen1.c
    UI/ui.c
    UI/components/ui_comp_hook.c
    UI/ui_helpers.c
    UI/images/ui_img_xmas_image_png.c
    UI/images/ui_img_clock_imge_png.c

    INCLUDE_DIRS "UI" "UI/components" "UI/fonts" "UI/images" "UI/screens" "."
    REQUIRES lvgl)

