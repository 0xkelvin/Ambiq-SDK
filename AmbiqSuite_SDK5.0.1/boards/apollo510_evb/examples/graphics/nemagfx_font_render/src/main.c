//*****************************************************************************
//
//! @file main.c
//!
//! @brief nemagfx_font_render main implementation.
//!
//! This example demonstrate 4 types of fonts in 1b, 2b, 4b, and 8b formats
//! with different alignment.
//!
//! AM_DEBUG_PRINTF
//! If enabled, debug messages will be sent over ITM.
//
//*****************************************************************************
//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************
#include "nemagfx_font_render.h"

//*****************************************************************************
//
// User configuration.
//
//*****************************************************************************
#ifndef USE_1BIT_FONT
#define USE_1BIT_FONT 1
#endif
#ifndef USE_2BIT_FONT
#define USE_2BIT_FONT 1
#endif
#ifndef USE_4BIT_FONT
#define USE_4BIT_FONT 1
#endif
#ifndef USE_8BIT_FONT
#define USE_8BIT_FONT 1
#endif

#define USE_BINS  0

#if USE_BINS
#define NEMA_FONT_LOAD_FROM_BIN
#endif

//*****************************************************************************
//
// Include the font files.
//
//*****************************************************************************

#define NEMA_FONT_IMPLEMENTATION

// set the NEMA_GPU_MEM macro to AM_SHARED_RW to locate the bitmap to the shared memory
#define NEMA_GPU_MEM const

#if USE_1BIT_FONT != 0
    #include "DejaVuSerif8pt1b.h"
#endif
#if USE_2BIT_FONT != 0
    #include "DejaVuSerif8pt2b.h"
#endif
#if USE_4BIT_FONT != 0
    #include "DejaVuSerif8pt4b.h"
#endif
#if USE_8BIT_FONT != 0
    #include "DejaVuSerif8pt8b.h"
#endif

#include "DejaVuSerif24pt4b.h"
#include "DejaVuSerif24pt4b_kern.h"

#undef NEMA_FONT_IMPLEMENTATION



//*****************************************************************************
//
// Variables Definition.
//
//*****************************************************************************
static img_obj_t g_sFB = {{0}, RESX, RESY, RESX*3, 0, NEMA_RGB24, 0};

//*****************************************************************************
//
// A demo function to load the bitmap bin files from the file system.
//
//*****************************************************************************
#if USE_BINS

#error "File system is not implemented in this example. This is \
just a demo to show how to load bin files from file system."
nema_buffer_t load_bin_file(const char *filename, int length, void *buffer)
{
    FILE * f = fopen (filename, "rb");
    nema_buffer_t bo = {0};

    if (f != NULL)
    {
        if (length < 0)
        {
            fseek (f, 0, SEEK_END);
            length = ftell (f);
            fseek (f, 0, SEEK_SET);
        }

        if ( !buffer )
        {
            bo = nema_buffer_create_pool(NEMA_MEM_POOL_ASSETS, length);
            buffer = nema_buffer_map(&bo);
        }

        if (buffer != NULL)
        {
            int size = fread (buffer, 1, length, f);
            if ( size != length )
            {
                printf("Cannot fread\n");
            }
        }
        else
        {
            printf("Cannot allocate buffer\n");
        }

        fclose (f);

        return bo;
    }
    else
    {
        printf("Can't open file %s\n", filename);
    }

    return bo;
}
#endif

//*****************************************************************************
//
//! @brief Initializes the frame buffer and loads the font bitmap.
//!
//! This function sets up the frame buffer and loads the font bitmap from MRAM
//！ or the filesystem. If there is a failure in memory allocation, the function
//！ will enter an infinite loop.
//
//*****************************************************************************
void load_objects(void)
{
    //
    // Load memory objects
    //
    g_sFB.bo = nema_buffer_create_pool(NEMA_MEM_POOL_FB, g_sFB.stride * (int32_t)g_sFB.h);
    (void)nema_buffer_map(&g_sFB.bo);

#if USE_BINS

#if USE_1BIT_FONT != 0
    DejaVuSerif8pt1b.bo = load_bin_file("DejaVuSerif8pt1b.bin", -1, 0);
#endif
#if USE_2BIT_FONT != 0
    DejaVuSerif8pt2b.bo = load_bin_file("DejaVuSerif8pt2b.bin", -1, 0);
#endif
#if USE_4BIT_FONT != 0
    DejaVuSerif8pt4b.bo = load_bin_file("DejaVuSerif8pt4b.bin", -1, 0);
#endif
#if USE_8BIT_FONT != 0
    DejaVuSerif8pt8b.bo = load_bin_file("DejaVuSerif8pt8b.bin", -1, 0);
#endif

DejaVuSerif24pt4b.bo = load_bin_file("DejaVuSerif24pt4b.bin", -1, 0);
DejaVuSerif24pt4b_kern.bo = load_bin_file("DejaVuSerif24pt4b_kern.bin", -1, 0);

#else

#if USE_1BIT_FONT != 0
    DejaVuSerif8pt1b.bo = nema_buffer_create_pool(NEMA_MEM_POOL_ASSETS, DejaVuSerif8pt1b.bitmap_size);
    if ( DejaVuSerif8pt1b.bo.base_virt == NULL )
    {
        am_util_stdio_printf("Failed to create buffer for font bitmap!\n");
        while(1);
    }
    (void)nema_buffer_map(&DejaVuSerif8pt1b.bo);
    (void)nema_memcpy(DejaVuSerif8pt1b.bo.base_virt, DejaVuSerif8pt1b.bitmap, (size_t)DejaVuSerif8pt1b.bitmap_size);
#endif

#if USE_2BIT_FONT != 0
    DejaVuSerif8pt2b.bo = nema_buffer_create_pool(NEMA_MEM_POOL_ASSETS, DejaVuSerif8pt2b.bitmap_size);
    if ( DejaVuSerif8pt2b.bo.base_virt == NULL )
    {
        am_util_stdio_printf("Failed to create buffer for font bitmap!\n");
        while(1);
    }
    (void)nema_buffer_map(&DejaVuSerif8pt2b.bo);
    (void)nema_memcpy(DejaVuSerif8pt2b.bo.base_virt, DejaVuSerif8pt2b.bitmap, (size_t)DejaVuSerif8pt2b.bitmap_size);
#endif

#if USE_4BIT_FONT != 0
    DejaVuSerif8pt4b.bo = nema_buffer_create_pool(NEMA_MEM_POOL_ASSETS, DejaVuSerif8pt4b.bitmap_size);
    if ( DejaVuSerif8pt4b.bo.base_virt == NULL )
    {
        am_util_stdio_printf("Failed to create buffer for font bitmap!\n");
        while(1);
    }
    (void)nema_buffer_map(&DejaVuSerif8pt4b.bo);
    (void)nema_memcpy(DejaVuSerif8pt4b.bo.base_virt, DejaVuSerif8pt4b.bitmap, (size_t)DejaVuSerif8pt4b.bitmap_size);
#endif

#if USE_8BIT_FONT != 0
    DejaVuSerif8pt8b.bo = nema_buffer_create_pool(NEMA_MEM_POOL_ASSETS, DejaVuSerif8pt8b.bitmap_size);
    if ( DejaVuSerif8pt8b.bo.base_virt == NULL )
    {
        am_util_stdio_printf("Failed to create buffer for font bitmap!\n");
        while(1);
    }
    (void)nema_buffer_map(&DejaVuSerif8pt8b.bo);
    (void)nema_memcpy(DejaVuSerif8pt8b.bo.base_virt, DejaVuSerif8pt8b.bitmap, (size_t)DejaVuSerif8pt8b.bitmap_size);
#endif

    DejaVuSerif24pt4b.bo = nema_buffer_create_pool(NEMA_MEM_POOL_ASSETS, DejaVuSerif24pt4b.bitmap_size);
    if ( DejaVuSerif24pt4b.bo.base_virt == NULL )
    {
        am_util_stdio_printf("Failed to create buffer for font bitmap!\n");
        while(1);
    }
    (void)nema_buffer_map(&DejaVuSerif24pt4b.bo);
    (void)nema_memcpy(DejaVuSerif24pt4b.bo.base_virt, DejaVuSerif24pt4b.bitmap, (size_t)DejaVuSerif24pt4b.bitmap_size);

    DejaVuSerif24pt4b_kern.bo = nema_buffer_create_pool(NEMA_MEM_POOL_ASSETS, DejaVuSerif24pt4b_kern.bitmap_size);
    if ( DejaVuSerif24pt4b_kern.bo.base_virt == NULL )
    {
        am_util_stdio_printf("Failed to create buffer for font bitmap!\n");
        while(1);
    }
    (void)nema_buffer_map(&DejaVuSerif24pt4b_kern.bo);
    (void)nema_memcpy(DejaVuSerif24pt4b_kern.bo.base_virt, DejaVuSerif24pt4b_kern.bitmap, (size_t)DejaVuSerif24pt4b_kern.bitmap_size);


#endif

}

//*****************************************************************************
//
//! @brief Draw characters.
//!
//! This function draws characters on the screen using the different font and
//! different alignment options.
//
//*****************************************************************************
void draw_characters(void)
{
    nema_cmdlist_t sCL;
    //
    // Create Command Lists
    //
    sCL = nema_cl_create();
    //
    // Bind Command List
    //
    nema_cl_bind(&sCL);
    //
    // Bind Framebuffer
    //
    nema_bind_dst_tex(g_sFB.bo.base_phys, g_sFB.w, g_sFB.h, g_sFB.format, g_sFB.stride);
    //
    // Set Clipping Rectangle
    //
    nema_set_clip(0, 0, RESX, RESY);
    nema_set_blend_fill(NEMA_BL_SRC);
    //
    // Fill Rectangle with Color
    //
    nema_fill_rect(0, 0, RESX, RESY, 0x10101010);

    char str[] = "0 bpp - Hello World!!!\n|-------------------------------|\nThe quick brown fox jumps     over the lazy dog!\n\
ThisIsAVeryVeryVeryVeryVeryVeryVeryVeryVeryVeryVeryVeryLongString\nH NemaGFX υποστηρίζει ΚΑΙ Unicode χαρακτήρες!!!";

    int w, h;

    const int32_t spacing = 5;

    nema_bind_font(&DejaVuSerif8pt1b);
    (void)nema_string_get_bbox(str, &w, &h, RESX / 2 - spacing * 3, 1);
    h += spacing;
    w += spacing;

    int32_t x, y;

    const int32_t xs[4] = {spacing, w + 2 * spacing, spacing, w + 2 * spacing};
    const int32_t ys[4] = {spacing, spacing, h + 2 * spacing, h + 2 * spacing};
    int32_t idx = 0;

    //
    // first a quarter of frame
    //
#if USE_1BIT_FONT != 0
    x = xs[idx];
    y = ys[idx];
    ++idx;

    str[0] = '1';

    nema_set_blend_fill(NEMA_BL_SRC);
    nema_fill_rect(x, y, w, h, 0x70707070U);
    nema_bind_font(&DejaVuSerif8pt1b);
    nema_print(str, x, y, w, h, 0xff00ff80U, NEMA_ALIGNX_LEFT | NEMA_TEXT_WRAP | NEMA_ALIGNY_TOP);

    nema_cl_submit(&sCL);
    (void)nema_cl_wait(&sCL);

    nema_cl_rewind(&sCL);
#endif
    //
    // second a quarter of frame
    //
#if USE_2BIT_FONT != 0
    x = xs[idx];
    y = ys[idx];
    ++idx;

    str[0] = '2';

    nema_set_blend_fill(NEMA_BL_SRC);
    nema_fill_rect(x, y, w, h, 0x70707070U);
    nema_bind_font(&DejaVuSerif8pt2b);
    nema_print(str, x, y, w, h, 0xff00ffffU, NEMA_ALIGNX_RIGHT | NEMA_TEXT_WRAP | NEMA_ALIGNY_BOTTOM);

    nema_cl_submit(&sCL);
    (void)nema_cl_wait(&sCL);

    nema_cl_rewind(&sCL);
#endif
    //
    // third a quarter of frame
    //
#if USE_4BIT_FONT != 0
    x = xs[idx];
    y = ys[idx];
    ++idx;

    str[0] = '4';

    nema_set_blend_fill(NEMA_BL_SRC);
    nema_fill_rect(x, y, w, h, 0x70707070U);
    nema_bind_font(&DejaVuSerif8pt4b);
    nema_print(str, x, y, w, h, 0xff0080ffU, NEMA_ALIGNX_CENTER | NEMA_TEXT_WRAP | NEMA_ALIGNY_CENTER);

    nema_cl_submit(&sCL);
    (void)nema_cl_wait(&sCL);

    nema_cl_rewind(&sCL);
#endif
    //
    // fourth a quarter of frame
    //
#if USE_8BIT_FONT != 0

    x = xs[idx];
    y = ys[idx];
    ++idx;

    str[0] = '8';

    nema_set_blend_fill(NEMA_BL_SRC);
    nema_fill_rect(x, y, w, h, 0x70707070U);
    nema_bind_font(&DejaVuSerif8pt8b);
    nema_print(str, x, y, w, h, 0x808ff08fU, NEMA_ALIGNX_JUSTIFY | NEMA_TEXT_WRAP | NEMA_ALIGNY_JUSTIFY);

    nema_cl_submit(&sCL);
    (void)nema_cl_wait(&sCL);

#endif

    // destroy command list
    nema_cl_destroy(&sCL);
}


//*****************************************************************************
//
//! @brief Draw characters with kerning effect.
//!
//! This function draws characters on the screen with kerning effect.
//
//*****************************************************************************
void draw_character_with_kerning(void)
{
    nema_cmdlist_t sCL;
    //
    // Create Command Lists
    //
    sCL = nema_cl_create();
    //
    // Bind Command List
    //
    nema_cl_bind(&sCL);
    //
    // Bind Framebuffer
    //
    nema_bind_dst_tex(g_sFB.bo.base_phys, g_sFB.w, g_sFB.h, g_sFB.format, g_sFB.stride);
    //
    // Set Clipping Rectangle
    //
    nema_set_clip(0, 0, RESX, RESY);
    nema_set_blend_fill(NEMA_BL_SRC);

    //
    // start for font display
    //
    nema_clear(0x10101010);

    nema_font_t *kern_font   = &DejaVuSerif24pt4b_kern;
    nema_font_t *simple_font = &DejaVuSerif24pt4b;

    char *str = "AV";
    nema_fill_rect(0, 0, 100, 50, 0x50505050);
    nema_fill_rect(0, 50, 100, 50, 0x50505050);

    nema_bind_font(simple_font);
    nema_print(str, 0, 0, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);
    nema_bind_font(kern_font);
    nema_print(str, 0, 50, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);

    str = "FA";
    nema_set_blend_fill(NEMA_BL_SRC);
    nema_fill_rect(105, 0, 100, 50, 0x50505050);
    nema_fill_rect(105, 50, 100, 50, 0x50505050);

    nema_bind_font(simple_font);
    nema_print(str, 105, 0, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);
    nema_bind_font(kern_font);
    nema_print(str, 105, 50, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);

    str = "Wö";
    nema_set_blend_fill(NEMA_BL_SRC);
    nema_fill_rect(210, 0, 100, 50, 0x50505050);
    nema_fill_rect(210, 50, 100, 50, 0x50505050);

    nema_bind_font(simple_font);
    nema_print(str, 210, 0, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);
    nema_bind_font(kern_font);
    nema_print(str, 210, 50, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);

    str = "Yu";
    nema_set_blend_fill(NEMA_BL_SRC);
    nema_fill_rect(315, 0, 100, 50, 0x50505050);
    nema_fill_rect(315, 50, 100, 50, 0x50505050);

    nema_bind_font(simple_font);
    nema_print(str, 315, 0, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);
    nema_bind_font(kern_font);
    nema_print(str, 315, 50, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);


    str = "Ya";
    nema_set_blend_fill(NEMA_BL_SRC);
    nema_fill_rect(0, 105, 100, 50, 0x50505050);
    nema_fill_rect(0, 155, 100, 50, 0x50505050);

    nema_bind_font(simple_font);
    nema_print(str, 0, 105, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);
    nema_bind_font(kern_font);
    nema_print(str, 0, 155, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);

    str = "Wa";
    nema_set_blend_fill(NEMA_BL_SRC);
    nema_fill_rect(105, 105, 100, 50, 0x50505050);
    nema_fill_rect(105, 155, 100, 50, 0x50505050);

    nema_bind_font(simple_font);
    nema_print(str, 105, 105, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);
    nema_bind_font(kern_font);
    nema_print(str, 105, 155, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);


    str = "Ye";
    nema_set_blend_fill(NEMA_BL_SRC);
    nema_fill_rect(210, 105, 100, 50, 0x50505050);
    nema_fill_rect(210, 155, 100, 50, 0x50505050);

    nema_bind_font(simple_font);
    nema_print(str, 210, 105, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);
    nema_bind_font(kern_font);
    nema_print(str, 210, 155, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);

    str = "FE";
    nema_set_blend_fill(NEMA_BL_SRC);
    nema_fill_rect(315, 105, 100, 50, 0x50505050);
    nema_fill_rect(315, 155, 100, 50, 0x50505050);

    nema_bind_font(simple_font);
    nema_print(str, 315, 105, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);
    nema_bind_font(kern_font);
    nema_print(str, 315, 155, 100, 50, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);

    str = "Testing Text Eng Sample";
    nema_set_blend_fill(NEMA_BL_SRC);
    nema_fill_rect(0, 215, 800, 60, 0x50505050);
    nema_fill_rect(0, 280, 800, 60, 0x50505050);

    nema_bind_font(simple_font);
    nema_print(str, 0, 215, 800, 60, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_TOP);
    nema_bind_font(kern_font);
    nema_print(str, 0, 280, 800, 60, 0xff00ff80U, NEMA_ALIGNX_CENTER | NEMA_ALIGNY_CENTER);

    nema_cl_submit(&sCL);
    (void)nema_cl_wait(&sCL);

    nema_cl_rewind(&sCL);

    nema_cl_destroy(&sCL);

}


//*****************************************************************************
//
//! @brief Four quarters display different fonts for every frame.
//!
//! This function display one type of font on one quaters,the whole screen could
//! demonstrate four types for every frame
//!
//! @return Zero.
//
//*****************************************************************************
int32_t font_render(void)
{
    //
    // Display font with different settings
    //
    draw_characters();

    //
    // transfer frame to the display
    //
    am_devices_display_transfer_frame(g_sFB.w,
                                      g_sFB.h,
                                      g_sFB.bo.base_phys,
                                      NULL, NULL);
    //
    // wait transfer done
    //
    am_devices_display_wait_transfer_done();

    am_util_delay_ms(5000);

    //
    // Display font with kerning effect
    //
    draw_character_with_kerning();

    //
    // transfer frame to the display
    //
    am_devices_display_transfer_frame(g_sFB.w,
                                      g_sFB.h,
                                      g_sFB.bo.base_phys,
                                      NULL, NULL);
    //
    // wait transfer done
    //
    am_devices_display_wait_transfer_done();

    am_util_delay_ms(5000);

    return 0;
}
