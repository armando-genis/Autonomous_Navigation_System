#include "generic_tasks/generic_tasks.h"
#include "generic_tasks/can_reset_task.h"

void canlib_init_generic_tasks(){
	init_can_reset_task();
}
