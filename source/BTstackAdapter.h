/*
 * BTstackAdapter.h
 *
 * Created on: 20.01.2020
 *
 * @author Sven Fabricius
 * Contact: Sven.Fabricius@livediesel.de
 */

#ifndef BTSTACKADAPTER_H_
#define BTSTACKADAPTER_H_

/* include required headers */
#include "BCDS_Retcode.h"

/**
 * @brief   Initializes the BTstack Adapter module.
 * @details   Initializes the BTstack Adapter module, i.e. required resources (task, semaphores, queues, etc.) will be created and initialized.
 *
 * @note
 * 1. This API shall be called only once.
 *
 * @return  RETCODE_OK on success, or an error code otherwise. Refer #Retcode_General_E for other values.
 */
Retcode_T BTstackAdapter_Init(void);

/**
 * @brief   Enabled the BTstack Adapter module.
 *
 * @note
 * 1. This API shall be called only once.
 *
 * @return  RETCODE_OK on success, or an error code otherwise. Refer #Retcode_General_E for other values.
 */
Retcode_T BTstackAdapter_Enable(void);

#endif /* BTSTACKADAPTER_H_ */
