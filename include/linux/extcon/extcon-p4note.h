/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef __LINUX_EXTCON_P4NOTE_H
#define __LINUX_EXTCON_P4NOTE_H

#include <linux/types.h>

struct extcon_dev;

int p4note_extcon_keyboard_feedback(struct extcon_dev *edev, bool connected);
int p4note_extcon_keyboard_ready(struct extcon_dev *edev);

#endif /* __LINUX_EXTCON_P4NOTE_H */
