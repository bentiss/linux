#ifndef _LINUX_PSMOUSE_H
#define _LINUX_PSMOUSE_H

#include <linux/serio.h>

void psmouse_overwrite_buttons(struct serio *serio, unsigned int mask);

#endif /* _LINUX_PSMOUSE_H */
