#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <alchemy/task.h>
#define TASK_PRIO 99
#define TASK_MODE 0
#define TASK_STKSZ 0

RT_TASK tA;

void compute_task_func (void *arg) {
	int i=0;
	for (i=0;i< 100 ;i++)
		printf("%d\n",i);
}


int main (int argc, char *argv[]) {
	int ret;
	mlockall(MCL_CURRENT|MCL_FUTURE);

	/*Création de la tâche*/
	ret = rt_task_create(&tA, "computeTask", TASK_STKSZ, TASK_PRIO, TASK_MODE);
	if( ret )
		perror("Impossible de créer la tâche ");

	/*Démarrage de la tâche*/
	ret = rt_task_start(&tA, &compute_task_func, NULL);
	if (ret) {
		perror("Démarrage de la tâche ");
		rt_task_delete(&tA);
		exit(1);
	}
	printf("Appuyer sur une touche pour terminer ....\n");
	getchar();
	/*Destruction de la tâche*/
	rt_task_delete(&tA);
	return 0;
}

