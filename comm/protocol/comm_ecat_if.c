#include "comm_ecat_if.h"

#include "9252_HW.h"
#include "applInterface.h"
#include "bsp_lan9252.h"
#include "cia402appl.h"
#include "ecatslv.h"

extern UINT16 CiA402_Init(void);

static uint8_t g_ecat_ready = 0U;
static uint8_t g_ecat_failed = 0U;

void comm_ecat_if_init(void)
{
    UINT16 init_err;

    if (g_ecat_ready || g_ecat_failed)
    {
        return;
    }

    bsp_lan9252_init_default();
    if (!bsp_lan9252_is_initialized())
    {
        g_ecat_failed = 1U;
        return;
    }

    if (HW_Init() != 0U)
    {
        g_ecat_failed = 1U;
        return;
    }

    init_err = MainInit();
    if (init_err != 0U)
    {
        g_ecat_failed = 1U;
        return;
    }

    (void)CiA402_Init();
    (void)APPL_GenerateMapping(&nPdInputSize, &nPdOutputSize);
    bRunApplication = TRUE;
    g_ecat_ready = 1U;
}

void comm_ecat_if_process(void)
{
    if (!g_ecat_ready)
    {
        return;
    }

    MainLoop();
}

uint8_t comm_ecat_if_is_ready(void)
{
    return g_ecat_ready;
}
