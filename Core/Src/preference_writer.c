/*
 * preference_writer.c
 *
 *  Created on: Apr 13, 2020
 *      Author: ben
 */

#include "preference_writer.h"

#include "flash_writer.h"
#include "main.h"
#include "user_config.h"
/*
PreferenceWriter::PreferenceWriter(uint32_t sector) {
    writer = new FlashWriter(sector);
    __sector = sector;
    __ready = false;
}
*/

void preference_writer_init(PreferenceWriter * pr, uint32_t sector)
{
  flash_writer_init(&pr->fw, sector);
  pr->sector = sector;
}

void preference_writer_open(PreferenceWriter * pr)
{
  flash_writer_open(&pr->fw);
  pr->ready = true;
}

bool preference_writer_ready(PreferenceWriter pr)
{
  return pr.ready;
}

void preference_writer_write_int(int x, int index)
{
  __int_reg[index] = x;
}

void preference_writer_write_float(float x, int index)
{
  __float_reg[index] = x;
}

/**
 * @brief Flushes the current preferences to flash memory.
 * This function writes all the integer and float registers to flash.
 * It should be called when the preferences are modified and need to be saved.
 */
void preference_writer_flush(PreferenceWriter * pr)
{
  int offs;
  for (offs = 0; offs < INT_REG_SIZE; offs++) {
    flash_writer_write_int(pr->fw, offs, __int_reg[offs]);
  }
  for (; offs < INT_REG_SIZE + FLOAT_REG_SIZE; offs++) {
    flash_writer_write_float(pr->fw, offs, __float_reg[offs - INT_REG_SIZE]);
  }
  pr->ready = false;
}

void preference_writer_load(PreferenceWriter pr)
{
  int offs;
  for (offs = 0; offs < INT_REG_SIZE; offs++) {
    __int_reg[offs] = flash_read_int(pr.fw, offs);
  }
  for (; offs < INT_REG_SIZE + FLOAT_REG_SIZE; offs++) {
    __float_reg[offs - INT_REG_SIZE] = flash_read_float(pr.fw, offs);
  }
}

void preference_writer_close(PreferenceWriter * pr)
{
  pr->ready = false;
  flash_writer_close(&pr->fw);
}
