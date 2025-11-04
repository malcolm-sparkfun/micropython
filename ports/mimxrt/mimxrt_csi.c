/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2025 Malcolm McKellips
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "py/runtime.h"
#include "py/mphal.h"
#include "modmimxrt.h"

#include "fsl_csi.h" 

// Following pattern from HSTX driver of rp2 port

// Default regs, see nxp_driver/sdk/drivers/csi/fsl_csi.c in CSI_Reset() and i.MX RT1060 Reference Manual Register Map
#define DEFAULT_CSI_CR1 (CSI_CR1_HSYNC_POL_MASK | CSI_CR1_EXT_VSYNC_MASK)
#define DEFAULT_CSI_CR2 (0u)
#define DEFAULT_CSI_CR3 (0u)

#if defined(CSI_CR18_CSI_LCDIF_BUFFER_LINES)
    #define DEFAULT_CSI_CR18 (CSI_CR18_AHB_HPROT(0x0DU) | CSI_CR18_CSI_LCDIF_BUFFER_LINES(0x02U))
#else
    #define DEFAULT_CSI_CR18 (CSI_CR18_AHB_HPROT(0x0DU))
#endif

#define DEFAULT_CSI_REG_FBUF_PARA (0u)
#define DEFAULT_CSI_REG_IMAG_PARA (0u)

// Base Objects
typedef struct _mimxrt_csi_obj_t {
    mp_obj_base_t base;
    // CSI_Type *csi_inst;
    // uint8_t csi_id;
} mimxrt_csi_obj_t;

typedef struct _mimxrt_csi_field_t {
    qstr name;
    uint8_t shift: 5; // max shift of 31
    uint8_t length: 6; // max 63 bits wide per field of each reg (even though 32 is the max reg size)
} mimxrt_csi_field_t;

static mimxrt_csi_field_t mimxrt_csi_cr1_fields_table[] = {
    { MP_QSTR_PIXEL_BIT, CSI_CR1_PIXEL_BIT_SHIFT, 1},
    { MP_QSTR_REDGE, CSI_CR1_REDGE_SHIFT, 1},
    { MP_QSTR_INV_PCLK, CSI_CR1_INV_PCLK_SHIFT, 1},
    { MP_QSTR_INV_DATA, CSI_CR1_INV_DATA_SHIFT, 1},
    { MP_QSTR_GCLK_MODE, CSI_CR1_GCLK_MODE_SHIFT, 1},
    { MP_QSTR_CLR_RXFIFO, CSI_CR1_CLR_RXFIFO_SHIFT, 1},
    { MP_QSTR_CLR_STATFIFO, CSI_CR1_CLR_STATFIFO_SHIFT, 1},
    { MP_QSTR_PACK_DIR, CSI_CR1_PACK_DIR_SHIFT, 1},
    { MP_QSTR_FCC, CSI_CR1_FCC_SHIFT, 1},
    { MP_QSTR_CCIR_EN, CSI_CR1_CCIR_EN_SHIFT, 1},
    { MP_QSTR_HSYNC_POL, CSI_CR1_HSYNC_POL_SHIFT, 1},
    { MP_QSTR_SOF_INTEN, CSI_CR1_SOF_INTEN_SHIFT, 1},
    { MP_QSTR_SOF_POL, CSI_CR1_SOF_POL_SHIFT, 1},
    { MP_QSTR_RXFF_INTEN, CSI_CR1_RXFF_INTEN_SHIFT, 1},
    { MP_QSTR_FB1_DMA_DONE_INTEN, CSI_CR1_FB1_DMA_DONE_INTEN_SHIFT, 1},
    { MP_QSTR_FB2_DMA_DONE_INTEN, CSI_CR1_FB2_DMA_DONE_INTEN_SHIFT, 1},
    { MP_QSTR_STATFF_INTEN, CSI_CR1_STATFF_INTEN_SHIFT, 1},
    { MP_QSTR_SFF_DMA_DONE_INTEN, CSI_CR1_SFF_DMA_DONE_INTEN_SHIFT, 1},
    { MP_QSTR_RF_OR_INTEN, CSI_CR1_RF_OR_INTEN_SHIFT, 1},
    { MP_QSTR_SF_OR_INTEN, CSI_CR1_SF_OR_INTEN_SHIFT, 1},
    { MP_QSTR_COF_INT_EN, CSI_CR1_COF_INT_EN_SHIFT, 1},
    { MP_QSTR_CCIR_MODE, CSI_CR1_CCIR_MODE_SHIFT, 1},
    { MP_QSTR_PrP_IF_EN, CSI_CR1_PrP_IF_EN_SHIFT, 1},
    { MP_QSTR_EOF_INT_EN, CSI_CR1_EOF_INT_EN_SHIFT, 1},
    { MP_QSTR_EXT_VSYNC, CSI_CR1_EXT_VSYNC_SHIFT, 1},
    { MP_QSTR_SWAP16_EN, CSI_CR1_SWAP16_EN_SHIFT, 1},
};

static const uint32_t mimxrt_csi_cr1_field_count = MP_ARRAY_SIZE(mimxrt_csi_cr1_fields_table);

static mimxrt_csi_field_t mimxrt_csi_cr2_fields_table[] = {
    { MP_QSTR_HSC, CSI_CR2_HSC_SHIFT, 8},
    {MP_QSTR_VSC, CSI_CR2_VSC_SHIFT, 8},
    { MP_QSTR_LVRM, CSI_CR2_LVRM_SHIFT, 3},
    { MP_QSTR_BTS, CSI_CR2_BTS_SHIFT, 2},
    { MP_QSTR_SCE, CSI_CR2_SCE_SHIFT, 1},
    { MP_QSTR_AFS, CSI_CR2_AFS_SHIFT, 2},
    { MP_QSTR_DRM, CSI_CR2_DRM_SHIFT, 1},
    { MP_QSTR_DMA_BURST_TYPE_SFF, CSI_CR2_DMA_BURST_TYPE_SFF_SHIFT, 2},
    { MP_QSTR_DMA_BURST_TYPE_RFF, CSI_CR2_DMA_BURST_TYPE_RFF_SHIFT, 2},
};

static const uint32_t mimxrt_csi_cr2_field_count = MP_ARRAY_SIZE(mimxrt_csi_cr2_fields_table);

static mimxrt_csi_field_t mimxrt_csi_cr3_fields_table[] = {
    { MP_QSTR_ECC_AUTO_EN, CSI_CR3_ECC_AUTO_EN_SHIFT, 1},
    { MP_QSTR_ECC_INT_EN, CSI_CR3_ECC_INT_EN_SHIFT, 1},
    { MP_QSTR_ZERO_PACK_EN, CSI_CR3_ZERO_PACK_EN_SHIFT, 1},
    { MP_QSTR_SENSOR_16BITS, CSI_CR3_SENSOR_16BITS_SHIFT, 1},
    { MP_QSTR_RxFF_LEVEL, CSI_CR3_RxFF_LEVEL_SHIFT, 3},
    { MP_QSTR_HRESP_ERR_EN, CSI_CR3_HRESP_ERR_EN_SHIFT, 1},
    { MP_QSTR_STATFF_LEVEL, CSI_CR3_STATFF_LEVEL_SHIFT, 3},
    { MP_QSTR_DMA_REQ_EN_SFF, CSI_CR3_DMA_REQ_EN_SFF_SHIFT, 1},
    { MP_QSTR_DMA_REQ_EN_RFF, CSI_CR3_DMA_REQ_EN_RFF_SHIFT, 1},
    { MP_QSTR_DMA_REFLASH_SFF, CSI_CR3_DMA_REFLASH_SFF_SHIFT, 1},
    { MP_QSTR_DMA_REFLASH_RFF, CSI_CR3_DMA_REFLASH_RFF_SHIFT, 1},
    { MP_QSTR_FRMCNT_RST, CSI_CR3_FRMCNT_RST_SHIFT, 1},
    { MP_QSTR_FRMCNT, CSI_CR3_FRMCNT_SHIFT, 16},
};

static const uint32_t mimxrt_csi_cr3_field_count = MP_ARRAY_SIZE(mimxrt_csi_cr3_fields_table);

static mimxrt_csi_field_t mimxrt_csi_rxcnt_fields_table[] = {
    { MP_QSTR_RXCNT, CSI_RXCNT_RXCNT_SHIFT, 22},
};

static const uint32_t mimxrt_csi_rxcnt_field_count = MP_ARRAY_SIZE(mimxrt_csi_rxcnt_fields_table);

static mimxrt_csi_field_t mimxrt_csi_sr_fields_table[] = {
    { MP_QSTR_DRDY, CSI_SR_DRDY_SHIFT, 1},
    { MP_QSTR_ECC_INT, CSI_SR_ECC_INT_SHIFT, 1},
    { MP_QSTR_HRESP_ERR_INT, CSI_SR_HRESP_ERR_INT_SHIFT, 1},
    { MP_QSTR_COF_INT, CSI_SR_COF_INT_SHIFT, 1},
    { MP_QSTR_F1_INT, CSI_SR_F1_INT_SHIFT, 1},
    { MP_QSTR_F2_INT, CSI_SR_F2_INT_SHIFT, 1},
    { MP_QSTR_SOF_INT, CSI_SR_SOF_INT_SHIFT, 1},
    { MP_QSTR_EOF_INT, CSI_SR_EOF_INT_SHIFT, 1},
    { MP_QSTR_RxFF_INT, CSI_SR_RxFF_INT_SHIFT, 1},
    { MP_QSTR_DMA_TSF_DONE_FB1, CSI_SR_DMA_TSF_DONE_FB1_SHIFT, 1},
    { MP_QSTR_DMA_TSF_DONE_FB2, CSI_SR_DMA_TSF_DONE_FB2_SHIFT, 1},
    { MP_QSTR_STATFF_INT, CSI_SR_STATFF_INT_SHIFT, 1},
    { MP_QSTR_DMA_TSF_DONE_SFF, CSI_SR_DMA_TSF_DONE_SFF_SHIFT, 1},
    { MP_QSTR_RF_OR_INT, CSI_SR_RF_OR_INT_SHIFT, 1},
    { MP_QSTR_SF_OR_INT, CSI_SR_SF_OR_INT_SHIFT, 1},
    { MP_QSTR_DMA_FIELD1_DONE, CSI_SR_DMA_FIELD1_DONE_SHIFT, 1},
    { MP_QSTR_DMA_FIELD0_DONE, CSI_SR_DMA_FIELD0_DONE_SHIFT, 1},
    { MP_QSTR_BASEADDR_CHHANGE_ERROR, CSI_SR_BASEADDR_CHHANGE_ERROR_SHIFT, 1},
};

static const uint32_t mimxrt_csi_sr_field_count = MP_ARRAY_SIZE(mimxrt_csi_sr_fields_table);

static mimxrt_csi_field_t mimxrt_csi_dmasa_statfifo_fields_table[] = {
    { MP_QSTR_DMA_START_ADDR_SFF, CSI_DMASA_STATFIFO_DMA_START_ADDR_SFF_SHIFT, 30},
};

static const uint32_t mimxrt_csi_dmasa_statfifo_field_count = MP_ARRAY_SIZE(mimxrt_csi_dmasa_statfifo_fields_table);

static mimxrt_csi_field_t mimxrt_csi_dmats_statfifo_fields_table[] = {
    { MP_QSTR_DMA_TSF_SIZE_SFF, CSI_DMATS_STATFIFO_DMA_TSF_SIZE_SFF_SHIFT, 32},
};

static const uint32_t mimxrt_csi_dmats_statfifo_field_count = MP_ARRAY_SIZE(mimxrt_csi_dmats_statfifo_fields_table);

static mimxrt_csi_field_t mimxrt_csi_dmasa_fb1_fields_table[] = {
    { MP_QSTR_DMA_START_ADDR_FB1, CSI_DMASA_FB1_DMA_START_ADDR_FB1_SHIFT, 30},
};

static const uint32_t mimxrt_csi_dmasa_fb1_field_count = MP_ARRAY_SIZE(mimxrt_csi_dmasa_fb1_fields_table);

static mimxrt_csi_field_t mimxrt_csi_dmasa_fb2_fields_table[] = {
    { MP_QSTR_DMA_START_ADDR_FB2, CSI_DMASA_FB2_DMA_START_ADDR_FB2_SHIFT, 30},
};

static const uint32_t mimxrt_csi_dmasa_fb2_field_count = MP_ARRAY_SIZE(mimxrt_csi_dmasa_fb2_fields_table);

static mimxrt_csi_field_t mimxrt_csi_fbuf_para_fields_table[] = {
    { MP_QSTR_FBUF_STRIDE_SHIFT, CSI_FBUF_PARA_FBUF_STRIDE_SHIFT, 16},
    { MP_QSTR_DEINTERLACE_STRIDE, CSI_FBUF_PARA_DEINTERLACE_STRIDE_SHIFT, 16},
};

static const uint32_t mimxrt_csi_fbuf_para_field_count = MP_ARRAY_SIZE(mimxrt_csi_fbuf_para_fields_table);

static mimxrt_csi_field_t mimxrt_csi_imag_para_fields_table[] = {
    { MP_QSTR_IMAGE_HEIGHT, CSI_IMAG_PARA_IMAGE_HEIGHT_SHIFT, 16},
    { MP_QSTR_IMAGE_WIDTH, CSI_IMAG_PARA_IMAGE_WIDTH_SHIFT, 16},
};

static const uint32_t mimxrt_csi_imag_para_field_count = MP_ARRAY_SIZE(mimxrt_csi_imag_para_fields_table);

static mimxrt_csi_field_t mimxrt_csi_cr18_fields_table[] = {
    { MP_QSTR_DEINTERLACE_EN, CSI_CR18_DEINTERLACE_EN_SHIFT, 1},
    { MP_QSTR_PARALLEL24_EN, CSI_CR18_PARALLEL24_EN_SHIFT, 1},
    { MP_QSTR_BASEADDR_SWITCH_EN, CSI_CR18_BASEADDR_SWITCH_EN_SHIFT, 1},
    { MP_QSTR_BASEADDR_SWITCH_SEL, CSI_CR18_BASEADDR_SWITCH_SEL_SHIFT, 1},
    { MP_QSTR_FIELD0_DONE_IE, CSI_CR18_FIELD0_DONE_IE_SHIFT, 1},
    { MP_QSTR_DMA_FIELD1_DONE_IE, CSI_CR18_DMA_FIELD1_DONE_IE_SHIFT, 1},
    { MP_QSTR_LAST_DMA_REQ_SEL, CSI_CR18_LAST_DMA_REQ_SEL_SHIFT, 1},
    { MP_QSTR_BASEADDR_CHANGE_ERROR_IE, CSI_CR18_BASEADDR_CHANGE_ERROR_IE_SHIFT, 1},
    { MP_QSTR_RGB888A_FORMAT_SEL, CSI_CR18_RGB888A_FORMAT_SEL_SHIFT, 1},
    { MP_QSTR_AHB_HPROT, CSI_CR18_AHB_HPROT_SHIFT, 4},
    { MP_QSTR_MASK_OPTION, CSI_CR18_MASK_OPTION_SHIFT, 2},
    { MP_QSTR_CSI_ENABLE, CSI_CR18_CSI_ENABLE_SHIFT, 1},
};

static const uint32_t mimxrt_csi_cr18_field_count = MP_ARRAY_SIZE(mimxrt_csi_cr18_fields_table);

static mimxrt_csi_field_t mimxrt_csi_cr19_fields_table[] = {
    { MP_QSTR_DMA_RFIFO_HIGHEST_FIFO_LEVEL, CSI_CR19_DMA_RFIFO_HIGHEST_FIFO_LEVEL_SHIFT, 3},
};

static const uint32_t mimxrt_csi_cr19_field_count = MP_ARRAY_SIZE(mimxrt_csi_cr19_fields_table);

// TODO: Lift these from the driver implementation?
void mimxrt_csi_init(void) {
    // Initialize CSI hardware here
}

void mimxrt_csi_deinit(void) {
    // Deinitialize CSI hardware here
}

/*********************************************************************/
// MIMXT object
static mimxrt_csi_obj_t mimxrt_csi_obj = {
    .base = { &mimxrt_csi_type }
};

static mp_obj_t mimxrt_csi_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    return MP_OBJ_FROM_PTR(&mimxrt_csi_obj);
}

// static mp_obj_t mimxrt_csi_foo(mp_obj_t self_in) {
//     // print("foo called\n");
//     // mp_printf("foo called\n");
//     mp_raise_msg_varg(&mp_type_OSError, MP_ERROR_TEXT("FOO CALLED"));
//     return mp_const_none;
// }
// static MP_DEFINE_CONST_FUN_OBJ_1(mimxrt_csi_foo_obj, mimxrt_csi_foo);

// ------------------------------- Register Setting and Reading Functions -------------------------------
// CSI.cr1([value])
static mp_obj_t mimxrt_csi_cr1(size_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        mp_int_t value = mp_obj_get_int(args[1]);
        CSI->CR1 = value;
        return mp_const_none;
    }
    return mp_obj_new_int_from_uint(CSI->CR1);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mimxrt_csi_cr1_obj, 1, 2, mimxrt_csi_cr1);

// CSI.cr2([value])
static mp_obj_t mimxrt_csi_cr2(size_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        mp_int_t value = mp_obj_get_int(args[1]);
        CSI->CR2 = value;
        return mp_const_none;
    }
    return mp_obj_new_int_from_uint(CSI->CR2);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mimxrt_csi_cr2_obj, 1, 2, mimxrt_csi_cr2);

// CSI.cr3([value])
static mp_obj_t mimxrt_csi_cr3(size_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        mp_int_t value = mp_obj_get_int(args[1]);
        CSI->CR3 = value;
        return mp_const_none;
    }
    return mp_obj_new_int_from_uint(CSI->CR3);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mimxrt_csi_cr3_obj, 1, 2, mimxrt_csi_cr3);

// CSI.statfifo() <Read only>
static mp_obj_t mimxrt_csi_statfifo(mp_obj_t self_in) {
    return mp_obj_new_int_from_uint(CSI->STATFIFO);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mimxrt_csi_statfifo_obj, mimxrt_csi_statfifo);

// CSI.rfifo() <>Read only>
static mp_obj_t mimxrt_csi_rfifo(mp_obj_t self_in) {
    return mp_obj_new_int_from_uint(CSI->RFIFO);
}
static MP_DEFINE_CONST_FUN_OBJ_1(mimxrt_csi_rfifo_obj, mimxrt_csi_rfifo);

// CSI.rxcnt([value])
static mp_obj_t mimxrt_csi_rxcnt(size_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        mp_int_t value = mp_obj_get_int(args[1]);
        CSI->RXCNT = value;
        return mp_const_none;
    }
    return mp_obj_new_int_from_uint(CSI->RXCNT);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mimxrt_csi_rxcnt_obj, 1, 2, mimxrt_csi_rxcnt);

// CSI.sr([value])
static mp_obj_t mimxrt_csi_sr(size_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        mp_int_t value = mp_obj_get_int(args[1]);
        CSI->SR = value;
        return mp_const_none;
    }
    return mp_obj_new_int_from_uint(CSI->SR);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mimxrt_csi_sr_obj, 1, 2, mimxrt_csi_sr);

// CSI.dmasa_statfifo([value])
static mp_obj_t mimxrt_csi_dmasa_statfifo(size_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        mp_int_t value = mp_obj_get_int(args[1]);
        CSI->DMASA_STATFIFO = value;
        return mp_const_none;
    }
    return mp_obj_new_int_from_uint(CSI->DMASA_STATFIFO);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mimxrt_csi_dmasa_statfifo_obj, 1, 2, mimxrt_csi_dmasa_statfifo);

// CSI.dmats_statfifo([value])
static mp_obj_t mimxrt_csi_dmats_statfifo(size_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        mp_int_t value = mp_obj_get_int(args[1]);
        CSI->DMATS_STATFIFO = value;
        return mp_const_none;
    }
    return mp_obj_new_int_from_uint(CSI->DMATS_STATFIFO);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mimxrt_csi_dmats_statfifo_obj, 1, 2, mimxrt_csi_dmats_statfifo);

// CSI.dmasa_fb1([value])
static mp_obj_t mimxrt_csi_dmasa_fb1(size_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        mp_int_t value = mp_obj_get_int(args[1]);
        CSI->DMASA_FB1 = value;
        return mp_const_none;
    }
    return mp_obj_new_int_from_uint(CSI->DMASA_FB1);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mimxrt_csi_dmasa_fb1_obj, 1, 2, mimxrt_csi_dmasa_fb1);

// CSI.dmasa_fb2([value])
static mp_obj_t mimxrt_csi_dmasa_fb2(size_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        mp_int_t value = mp_obj_get_int(args[1]);
        CSI->DMASA_FB2 = value;
        return mp_const_none;
    }
    return mp_obj_new_int_from_uint(CSI->DMASA_FB2);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mimxrt_csi_dmasa_fb2_obj, 1, 2, mimxrt_csi_dmasa_fb2);

// CSI.fbuf_para([value])
static mp_obj_t mimxrt_csi_fbuf_para(size_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        mp_int_t value = mp_obj_get_int(args[1]);
        CSI->FBUF_PARA = value;
        return mp_const_none;
    }
    return mp_obj_new_int_from_uint(CSI->FBUF_PARA);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mimxrt_csi_fbuf_para_obj, 1, 2, mimxrt_csi_fbuf_para);

// CSI.imag_para([value])
static mp_obj_t mimxrt_csi_imag_para(size_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        mp_int_t value = mp_obj_get_int(args[1]);
        CSI->IMAG_PARA = value;
        return mp_const_none;
    }
    return mp_obj_new_int_from_uint(CSI->IMAG_PARA);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mimxrt_csi_imag_para_obj, 1, 2, mimxrt_csi_imag_para);

// CSI.cr18([value])
static mp_obj_t mimxrt_csi_cr18(size_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        mp_int_t value = mp_obj_get_int(args[1]);
        CSI->CR18 = value;
        return mp_const_none;
    }
    return mp_obj_new_int_from_uint(CSI->CR18);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mimxrt_csi_cr18_obj, 1, 2, mimxrt_csi_cr18);

// CSI.cr19([value])
static mp_obj_t mimxrt_csi_cr19(size_t n_args, const mp_obj_t *args) {
    if (n_args > 1) {
        mp_int_t value = mp_obj_get_int(args[1]);
        CSI->CR19 = value;
        return mp_const_none;
    }
    return mp_obj_new_int_from_uint(CSI->CR19);
}
static MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mimxrt_csi_cr19_obj, 1, 2, mimxrt_csi_cr19);

static mp_obj_t mimxrt_csi_pack_reg(size_t n_pos_args, mp_map_t *kw_args, mp_uint_t default_value, mimxrt_csi_field_t *fields_table, uint32_t field_count) {
    // Pack keyword settings into a control register value
    mp_uint_t value = default_value;

    if (n_pos_args > 1) {
        mp_raise_TypeError(MP_ERROR_TEXT("only keyword arguments are allowed"));
    }
    mp_uint_t remaining = kw_args->used;

    mp_map_elem_t *default_entry = mp_map_lookup(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_default), MP_MAP_LOOKUP);
    if (default_entry) {
        remaining--;
        value = mp_obj_get_int_truncated(default_entry->value);
    }

    for (mp_uint_t i = 0; i < field_count; i++) {
        mp_map_elem_t *field_entry = mp_map_lookup(
            kw_args,
            MP_OBJ_NEW_QSTR(fields_table[i].name),
            MP_MAP_LOOKUP
            );
        if (field_entry) {
            remaining--;
            mp_uint_t field_value = mp_obj_get_int_truncated(field_entry->value);
            mp_uint_t mask = ((1 << fields_table[i].length) - 1);
            mp_uint_t masked_value = field_value & mask;
            if (field_value != masked_value) {
                mp_raise_ValueError(MP_ERROR_TEXT("bad field value"));
            }
            value &= ~(mask << fields_table[i].shift);
            value |= masked_value << fields_table[i].shift;
        }
    }

    if (remaining) {
        mp_raise_TypeError(NULL);
    }

    return mp_obj_new_int_from_uint(value);
}

static mp_obj_t mimxrt_csi_unpack_reg(mp_obj_t value_obj, mimxrt_csi_field_t *fields_table, uint32_t field_count) {
    // Return a dict representing the unpacked fields of a control register value
    mp_obj_t result_dict[field_count * 2];

    mp_uint_t value = mp_obj_get_int_truncated(value_obj);

    for (mp_uint_t i = 0; i < field_count; i++) {
        result_dict[i * 2] = MP_OBJ_NEW_QSTR(fields_table[i].name);
        mp_uint_t field_value =
            (value >> fields_table[i].shift) & ((1 << fields_table[i].length) - 1);
        result_dict[i * 2 + 1] = MP_OBJ_NEW_SMALL_INT(field_value);
    }

    return mp_obj_dict_make_new(&mp_type_dict, 0, field_count, result_dict);
}

// ------------------------------- Packing Functions (CSI.pack_...) ------------------------------- 
static mp_obj_t mimxrt_csi_pack_cr1(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    return mimxrt_csi_pack_reg(n_args, kw_args, DEFAULT_CSI_CR1, mimxrt_csi_cr1_fields_table, mimxrt_csi_cr1_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mimxrt_csi_pack_cr1_obj, 1, mimxrt_csi_pack_cr1);

static mp_obj_t mimxrt_csi_pack_cr2(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    return mimxrt_csi_pack_reg(n_args, kw_args, DEFAULT_CSI_CR2, mimxrt_csi_cr2_fields_table, mimxrt_csi_cr2_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mimxrt_csi_pack_cr2_obj, 1, mimxrt_csi_pack_cr2);

static mp_obj_t mimxrt_csi_pack_cr3(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    return mimxrt_csi_pack_reg(n_args, kw_args, DEFAULT_CSI_CR3, mimxrt_csi_cr3_fields_table, mimxrt_csi_cr3_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mimxrt_csi_pack_cr3_obj, 1, mimxrt_csi_pack_cr3);

static mp_obj_t mimxrt_csi_pack_rxcnt(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    return mimxrt_csi_pack_reg(n_args, kw_args, 0u, mimxrt_csi_rxcnt_fields_table, mimxrt_csi_rxcnt_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mimxrt_csi_pack_rxcnt_obj, 1, mimxrt_csi_pack_rxcnt);

static mp_obj_t mimxrt_csi_pack_sr(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    return mimxrt_csi_pack_reg(n_args, kw_args, 0u, mimxrt_csi_sr_fields_table, mimxrt_csi_sr_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mimxrt_csi_pack_sr_obj, 1, mimxrt_csi_pack_sr);

static mp_obj_t mimxrt_csi_pack_dmasa_statfifo(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    return mimxrt_csi_pack_reg(n_args, kw_args, 0u, mimxrt_csi_dmasa_statfifo_fields_table, mimxrt_csi_dmasa_statfifo_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mimxrt_csi_pack_dmasa_statfifo_obj, 1, mimxrt_csi_pack_dmasa_statfifo);

static mp_obj_t mimxrt_csi_pack_dmats_statfifo(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    return mimxrt_csi_pack_reg(n_args, kw_args, 0u, mimxrt_csi_dmats_statfifo_fields_table, mimxrt_csi_dmats_statfifo_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mimxrt_csi_pack_dmats_statfifo_obj, 1, mimxrt_csi_pack_dmats_statfifo);

static mp_obj_t mimxrt_csi_pack_dmasa_fb1(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    return mimxrt_csi_pack_reg(n_args, kw_args, 0u, mimxrt_csi_dmasa_fb1_fields_table, mimxrt_csi_dmasa_fb1_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mimxrt_csi_pack_dmasa_fb1_obj, 1, mimxrt_csi_pack_dmasa_fb1);

static mp_obj_t mimxrt_csi_pack_dmasa_fb2(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    return mimxrt_csi_pack_reg(n_args, kw_args, 0u, mimxrt_csi_dmasa_fb2_fields_table, mimxrt_csi_dmasa_fb2_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mimxrt_csi_pack_dmasa_fb2_obj, 1, mimxrt_csi_pack_dmasa_fb2);

static mp_obj_t mimxrt_csi_pack_fbuf_para(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    return mimxrt_csi_pack_reg(n_args, kw_args, DEFAULT_CSI_REG_FBUF_PARA, mimxrt_csi_fbuf_para_fields_table, mimxrt_csi_fbuf_para_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mimxrt_csi_pack_fbuf_para_obj, 1, mimxrt_csi_pack_fbuf_para);

static mp_obj_t mimxrt_csi_pack_imag_para(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    return mimxrt_csi_pack_reg(n_args, kw_args, DEFAULT_CSI_REG_IMAG_PARA, mimxrt_csi_imag_para_fields_table, mimxrt_csi_imag_para_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mimxrt_csi_pack_imag_para_obj, 1, mimxrt_csi_pack_imag_para);

static mp_obj_t mimxrt_csi_pack_cr18(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    return mimxrt_csi_pack_reg(n_args, kw_args, DEFAULT_CSI_CR18, mimxrt_csi_cr18_fields_table, mimxrt_csi_cr18_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mimxrt_csi_pack_cr18_obj, 1, mimxrt_csi_pack_cr18);

static mp_obj_t mimxrt_csi_pack_cr19(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    return mimxrt_csi_pack_reg(n_args, kw_args, 0u, mimxrt_csi_cr19_fields_table, mimxrt_csi_cr19_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_KW(mimxrt_csi_pack_cr19_obj, 1, mimxrt_csi_pack_cr19);

// -------------------------------  Unpacking functions (CSI.unpack_...) ------------------------------- 
static mp_obj_t mimxrt_csi_unpack_cr1(mp_obj_t self_in, mp_obj_t value_obj) {
    return mimxrt_csi_unpack_reg(value_obj, mimxrt_csi_cr1_fields_table, mimxrt_csi_cr1_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mimxrt_csi_unpack_cr1_obj, mimxrt_csi_unpack_cr1);

static mp_obj_t mimxrt_csi_unpack_cr2(mp_obj_t self_in, mp_obj_t value_obj) {
    return mimxrt_csi_unpack_reg(value_obj, mimxrt_csi_cr2_fields_table, mimxrt_csi_cr2_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mimxrt_csi_unpack_cr2_obj, mimxrt_csi_unpack_cr2);

static mp_obj_t mimxrt_csi_unpack_cr3(mp_obj_t self_in, mp_obj_t value_obj) {
    return mimxrt_csi_unpack_reg(value_obj, mimxrt_csi_cr3_fields_table, mimxrt_csi_cr3_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mimxrt_csi_unpack_cr3_obj, mimxrt_csi_unpack_cr3);

static mp_obj_t mimxrt_csi_unpack_rxcnt(mp_obj_t self_in, mp_obj_t value_obj) {
    return mimxrt_csi_unpack_reg(value_obj, mimxrt_csi_rxcnt_fields_table, mimxrt_csi_rxcnt_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mimxrt_csi_unpack_rxcnt_obj, mimxrt_csi_unpack_rxcnt);

static mp_obj_t mimxrt_csi_unpack_sr(mp_obj_t self_in, mp_obj_t value_obj) {
    return mimxrt_csi_unpack_reg(value_obj, mimxrt_csi_sr_fields_table, mimxrt_csi_sr_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mimxrt_csi_unpack_sr_obj, mimxrt_csi_unpack_sr);

static mp_obj_t mimxrt_csi_unpack_dmasa_statfifo(mp_obj_t self_in, mp_obj_t value_obj) {
    return mimxrt_csi_unpack_reg(value_obj, mimxrt_csi_dmasa_statfifo_fields_table, mimxrt_csi_dmasa_statfifo_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mimxrt_csi_unpack_dmasa_statfifo_obj, mimxrt_csi_unpack_dmasa_statfifo);

static mp_obj_t mimxrt_csi_unpack_dmats_statfifo(mp_obj_t self_in, mp_obj_t value_obj) {
    return mimxrt_csi_unpack_reg(value_obj, mimxrt_csi_dmats_statfifo_fields_table, mimxrt_csi_dmats_statfifo_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mimxrt_csi_unpack_dmats_statfifo_obj, mimxrt_csi_unpack_dmats_statfifo);

static mp_obj_t mimxrt_csi_unpack_dmasa_fb1(mp_obj_t self_in, mp_obj_t value_obj) {
    return mimxrt_csi_unpack_reg(value_obj, mimxrt_csi_dmasa_fb1_fields_table, mimxrt_csi_dmasa_fb1_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mimxrt_csi_unpack_dmasa_fb1_obj, mimxrt_csi_unpack_dmasa_fb1);

static mp_obj_t mimxrt_csi_unpack_dmasa_fb2(mp_obj_t self_in, mp_obj_t value_obj) {
    return mimxrt_csi_unpack_reg(value_obj, mimxrt_csi_dmasa_fb2_fields_table, mimxrt_csi_dmasa_fb2_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mimxrt_csi_unpack_dmasa_fb2_obj, mimxrt_csi_unpack_dmasa_fb2);

static mp_obj_t mimxrt_csi_unpack_fbuf_para(mp_obj_t self_in, mp_obj_t value_obj) {
    return mimxrt_csi_unpack_reg(value_obj, mimxrt_csi_fbuf_para_fields_table, mimxrt_csi_fbuf_para_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mimxrt_csi_unpack_fbuf_para_obj, mimxrt_csi_unpack_fbuf_para);

static mp_obj_t mimxrt_csi_unpack_imag_para(mp_obj_t self_in, mp_obj_t value_obj) {
    return mimxrt_csi_unpack_reg(value_obj, mimxrt_csi_imag_para_fields_table, mimxrt_csi_imag_para_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mimxrt_csi_unpack_imag_para_obj, mimxrt_csi_unpack_imag_para);

static mp_obj_t mimxrt_csi_unpack_cr18(mp_obj_t self_in, mp_obj_t value_obj) {
    return mimxrt_csi_unpack_reg(value_obj, mimxrt_csi_cr18_fields_table, mimxrt_csi_cr18_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mimxrt_csi_unpack_cr18_obj, mimxrt_csi_unpack_cr18);

static mp_obj_t mimxrt_csi_unpack_cr19(mp_obj_t self_in, mp_obj_t value_obj) {
    return mimxrt_csi_unpack_reg(value_obj, mimxrt_csi_cr19_fields_table, mimxrt_csi_cr19_field_count);
}
static MP_DEFINE_CONST_FUN_OBJ_2(mimxrt_csi_unpack_cr19_obj, mimxrt_csi_unpack_cr19);

static const mp_rom_map_elem_t mimxrt_csi_locals_dict_table[] = {
    // { MP_ROM_QSTR(MP_QSTR_foo), MP_ROM_PTR(&mimxrt_csi_foo_obj) },
    // register accessors
    { MP_ROM_QSTR(MP_QSTR_cr1), MP_ROM_PTR(&mimxrt_csi_cr1_obj) },
    { MP_ROM_QSTR(MP_QSTR_cr2), MP_ROM_PTR(&mimxrt_csi_cr2_obj) },
    { MP_ROM_QSTR(MP_QSTR_cr3), MP_ROM_PTR(&mimxrt_csi_cr3_obj) },
    { MP_ROM_QSTR(MP_QSTR_statfifo), MP_ROM_PTR(&mimxrt_csi_statfifo_obj) },
    { MP_ROM_QSTR(MP_QSTR_rfifo), MP_ROM_PTR(&mimxrt_csi_rfifo_obj) },
    { MP_ROM_QSTR(MP_QSTR_rxcnt), MP_ROM_PTR(&mimxrt_csi_rxcnt_obj) },
    { MP_ROM_QSTR(MP_QSTR_sr), MP_ROM_PTR(&mimxrt_csi_sr_obj) },
    { MP_ROM_QSTR(MP_QSTR_dmasa_statfifo), MP_ROM_PTR(&mimxrt_csi_dmasa_statfifo_obj) },
    { MP_ROM_QSTR(MP_QSTR_dmats_statfifo), MP_ROM_PTR(&mimxrt_csi_dmats_statfifo_obj) },
    { MP_ROM_QSTR(MP_QSTR_dmasa_fb1), MP_ROM_PTR(&mimxrt_csi_dmasa_fb1_obj) },
    { MP_ROM_QSTR(MP_QSTR_dmasa_fb2), MP_ROM_PTR(&mimxrt_csi_dmasa_fb2_obj) },
    { MP_ROM_QSTR(MP_QSTR_fbuf_para), MP_ROM_PTR(&mimxrt_csi_fbuf_para_obj) },
    { MP_ROM_QSTR(MP_QSTR_imag_para), MP_ROM_PTR(&mimxrt_csi_imag_para_obj) },
    { MP_ROM_QSTR(MP_QSTR_cr18), MP_ROM_PTR(&mimxrt_csi_cr18_obj) },
    { MP_ROM_QSTR(MP_QSTR_cr19), MP_ROM_PTR(&mimxrt_csi_cr19_obj) },

    // packing functions
    { MP_ROM_QSTR(MP_QSTR_pack_cr1), MP_ROM_PTR(&mimxrt_csi_pack_cr1_obj) },
    { MP_ROM_QSTR(MP_QSTR_pack_cr2), MP_ROM_PTR(&mimxrt_csi_pack_cr2_obj) },
    { MP_ROM_QSTR(MP_QSTR_pack_cr3), MP_ROM_PTR(&mimxrt_csi_pack_cr3_obj) },
    { MP_ROM_QSTR(MP_QSTR_pack_rxcnt), MP_ROM_PTR(&mimxrt_csi_pack_rxcnt_obj) },
    { MP_ROM_QSTR(MP_QSTR_pack_sr), MP_ROM_PTR(&mimxrt_csi_pack_sr_obj) },
    { MP_ROM_QSTR(MP_QSTR_pack_dmasa_statfifo), MP_ROM_PTR(&mimxrt_csi_pack_dmasa_statfifo_obj) },
    { MP_ROM_QSTR(MP_QSTR_pack_dmats_statfifo), MP_ROM_PTR(&mimxrt_csi_pack_dmats_statfifo_obj) },
    { MP_ROM_QSTR(MP_QSTR_pack_dmasa_fb1), MP_ROM_PTR(&mimxrt_csi_pack_dmasa_fb1_obj) },
    { MP_ROM_QSTR(MP_QSTR_pack_dmasa_fb2), MP_ROM_PTR(&mimxrt_csi_pack_dmasa_fb2_obj) },
    { MP_ROM_QSTR(MP_QSTR_pack_fbuf_para), MP_ROM_PTR(&mimxrt_csi_pack_fbuf_para_obj) },
    { MP_ROM_QSTR(MP_QSTR_pack_imag_para), MP_ROM_PTR(&mimxrt_csi_pack_imag_para_obj) },
    { MP_ROM_QSTR(MP_QSTR_pack_cr18), MP_ROM_PTR(&mimxrt_csi_pack_cr18_obj) },
    { MP_ROM_QSTR(MP_QSTR_pack_cr19), MP_ROM_PTR(&mimxrt_csi_pack_cr19_obj) },

    // unpacking functions
    { MP_ROM_QSTR(MP_QSTR_unpack_cr1), MP_ROM_PTR(&mimxrt_csi_unpack_cr1_obj) },
    { MP_ROM_QSTR(MP_QSTR_unpack_cr2), MP_ROM_PTR(&mimxrt_csi_unpack_cr2_obj) },
    { MP_ROM_QSTR(MP_QSTR_unpack_cr3), MP_ROM_PTR(&mimxrt_csi_unpack_cr3_obj) },
    { MP_ROM_QSTR(MP_QSTR_unpack_rxcnt), MP_ROM_PTR(&mimxrt_csi_unpack_rxcnt_obj) },
    { MP_ROM_QSTR(MP_QSTR_unpack_sr), MP_ROM_PTR(&mimxrt_csi_unpack_sr_obj) },
    { MP_ROM_QSTR(MP_QSTR_unpack_dmasa_statfifo), MP_ROM_PTR(&mimxrt_csi_unpack_dmasa_statfifo_obj) },
    { MP_ROM_QSTR(MP_QSTR_unpack_dmats_statfifo), MP_ROM_PTR(&mimxrt_csi_unpack_dmats_statfifo_obj) },
    { MP_ROM_QSTR(MP_QSTR_unpack_dmasa_fb1), MP_ROM_PTR(&mimxrt_csi_unpack_dmasa_fb1_obj) },
    { MP_ROM_QSTR(MP_QSTR_unpack_dmasa_fb2), MP_ROM_PTR(&mimxrt_csi_unpack_dmasa_fb2_obj) },
    { MP_ROM_QSTR(MP_QSTR_unpack_fbuf_para), MP_ROM_PTR(&mimxrt_csi_unpack_fbuf_para_obj) },
    { MP_ROM_QSTR(MP_QSTR_unpack_imag_para), MP_ROM_PTR(&mimxrt_csi_unpack_imag_para_obj) },
    { MP_ROM_QSTR(MP_QSTR_unpack_cr18), MP_ROM_PTR(&mimxrt_csi_unpack_cr18_obj) },
    { MP_ROM_QSTR(MP_QSTR_unpack_cr19), MP_ROM_PTR(&mimxrt_csi_unpack_cr19_obj) },
};

static MP_DEFINE_CONST_DICT(mimxrt_csi_locals_dict, mimxrt_csi_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    mimxrt_csi_type,
    MP_QSTR_CSI,
    MP_TYPE_FLAG_NONE,
    make_new, mimxrt_csi_make_new,
    locals_dict, &mimxrt_csi_locals_dict
);

