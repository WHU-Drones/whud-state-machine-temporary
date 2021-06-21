# An simple tutorial for WhudStateMachine

In order to simply use WhudStateMachine, we just need to configure the files in folder **config**.

<img src="images\image-20210612121901791.png" alt="image-20210612121901791" style="zoom: 67%;" />

We can see three files in **config** folder. As main_task.yaml contains main tasks that are executed sequentially, interrupt_task.yaml contains interrupt tasks for each main task. plugin_params.yaml contains parameters for plugins. We first take a look at how to set main tasks.

## Set main tasks

Let's open main_task.yaml first.

<img src="images\image-20210612123032421.png" alt="image-20210612123032421" style="zoom: 80%;" />

Now, let's break the code down.

<img src="images\image-20210612123945198.png" alt="image-20210612123945198" style="zoom:80%;" />

If we want to add some main tasks(i.e. task_A and task_B), we can write the names of tasks in format as line 10 and 11. Note that task_A and task_B are only the names specified by users, their corresponding plugins and parameters will be defined following.

<img src="images\image-20210612124039207.png" alt="image-20210612124039207" style="zoom:80%;" />

Here is the description for our main tasks. Line 16 is the name of task which we are going to configure(**must be same as names defined in main_task_list**). Line 17 specifies the  plugin type of task. Line 18 is the maximum executing time for this task(**in seconds and must be integer**). When time is finished it will jump to the next task. Line 19 to line 21 specify the parameters needed for plugins(**which are different for different plugins**). Line 22 is the task name which must be same as name defined in main_task_list and main task description(line 10 and line 16). Line 23 is the name of interrupt task. Just like the name of main task, it's also a user-defined name. Its detailed description is in interrupt_task.yaml. 

## Set interrupt task

Let's open the interrupt_task.yaml

<img src="images\image-20210614234549948.png" alt="image-20210614234549948" />

It's almost the same as the main_task.yaml, with only one difference in line 10:

<img src="images\image-20210614234659188.png" alt="image-20210614234659188" />

 We assume that when interrupt task is done, it can decide which main task it will jump to, and the **return_name** is the name of task which will be jumped to  when interrupt task is done.  It also has this probability that interrupt task is not done when time is finished. In this case it will jump to the main task in which the interrupt task is stimulated, and interrupt task is disabled(it will not be stimulated again) until next main task.