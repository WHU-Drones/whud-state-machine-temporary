# An simple tutorial for WhudStateMachine

For simply use WhudStateMachine, we just need to configure the files in folder **config**.

<img src="C:\Users\admin\AppData\Roaming\Typora\typora-user-images\image-20210612121901791.png" alt="image-20210612121901791" style="zoom: 67%;" />

We can see three files in **config** folder. As main_task.yaml contains main tasks that are executed sequentially, interrupt_task.yaml contains interrupt tasks for each main task. plugin_params.yaml contains parameters for plugins. We first take a look at the main_task.yaml.

<img src="C:\Users\admin\AppData\Roaming\Typora\typora-user-images\image-20210612123032421.png" alt="image-20210612123032421" style="zoom: 80%;" />

Now, let's break the code down.

<img src="C:\Users\admin\AppData\Roaming\Typora\typora-user-images\image-20210612123945198.png" alt="image-20210612123945198" style="zoom:80%;" />

If we want to add some main tasks(i.e. task_A and task_B), we can write the names of tasks in format as line 9 and 10. Note that task_A and task_B are only the names specified by users, which have no relationship with the plugins' names.

<img src="C:\Users\admin\AppData\Roaming\Typora\typora-user-images\image-20210612124039207.png" alt="image-20210612124039207" style="zoom:80%;" />

Here is the description for our main tasks. Line 16 is the name of task which we are going to configure. Line 17 specifies the  plugin type of task. Line 18 is the maximum 

executing time(in seconds and must be integer) for this task. After this time will jump to the next task. Line 19 to line 21 specify the parameters needed for plugins(which may change for different plugins). Line 22 is the task name which must be same as name defined in main_task_list and main task description(line 10 and line 16). Line 23 is the name of interrupt task. Just like the name of main task, it's also a user-defined name. Its description is in interrupt_task.yaml. 