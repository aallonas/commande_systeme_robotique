#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#define TASK_PRIO 99
#define TASK_MODE 0
#define TASK_STKSZ 0

RT_TASK periodic_task;

/*periode de la tâche en ns (ici 100 ms)*/
long long period_ns = 100*1000*1000LL;

/*fonction appelée périodiquement associée à la tâche "periodic task"*/
void periodic_task_func (void *arg) {
	int i;
	RTIME start_ns; /* Instant de démarrage de la tâche périodique*/
	/*Lecture du timer courant et ajout 1 ms*/
	start_ns = rt_timer_read()+1000000;
	/*Bascule la tâche courante en tâche périodique de période spécifiée*/
	rt_task_set_periodic(NULL, start_ns, period_ns);
	while(1)
	{
		/*Attente du prochaine top horloge
		cadencé à 100 ms (restitue la main à Linux)*/
		rt_task_wait_period(NULL);
		i++;
		printf("%d\n",i);
	}
}

int main (int argc, char *argv[]) {
	int ret;
	mlockall(MCL_CURRENT|MCL_FUTURE);
	ret = rt_task_create(&tA, "computeTask", TASK_STKSZ, TASK_PRIO, TASK_MODE);
	if( ret )
		perror("Impossible de créer la tâche ");
	ret = rt_task_start(&tA, &periodic_task_func, NULL);
	if (ret) {
		perror("Démarrage de la tâche ");
		rt_task_delete(&tA);
		exit(1);
	}
	getchar();
	rt_task_delete(&tA);
	return 0;
}
