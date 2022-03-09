# This is a sample Python script.
# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import Experiment01 as E1
import Experiment02 as E2
import Experiment03 as E3
import Experiment05 as E5
import time
def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


if __name__ == '__main__':
    print_hi(__name__)
    

"""
Iterations_amount = 57
Ex02_times = [0]*Iterations_amount
Ex03_times = [0]*Iterations_amount
Average1 = E1.EXPERIMENT01()
Average2 = 0
c_time = time.time()
for index in range(Iterations_amount):
    Ex02_times[index] = E2.EXPERIMENT02()
    print(Ex02_times[index])
    Average2 = Average2 + round((Ex02_times[index]/Iterations_amount),4)
    Finished = (index+1)/Iterations_amount
    print('\n')
    print('Finished [%]: ', 100*Finished,'%')
    c_time2 = time.time()
    print('Estimated remaining time: ', (c_time2-c_time)*(-1+1/Finished))
    print('\n')

print('Experiment 01:')
print('Average: ', Average1)
print('Max: ', Average1)
print("Min: ", Average1)
print('\n')
print('Experiment 02:')
print('Average: ', Average2)
print('Max: ', max(Ex02_times))
print("Min: ", min(Ex02_times))
print('\n')

Iterations_amount = 30
Ex03_times = [0]*Iterations_amount
Average3 = 0
c_time = time.time()
for index in range(Iterations_amount):
    Ex03_times[index] = E3.EXPERIMENT03()
    print(Ex03_times[index])
    Average3 = Average3 + round((Ex03_times[index]/Iterations_amount),4)
    Finished = (index+1)/Iterations_amount
    print('Finished [%]: ', 100*Finished, '%')
    c_time2 = time.time()
    print('Estimated remaining time: ', (c_time2-c_time)*(-1+1/Finished))

print('Experiment 01:')
print('Average: ', Average1)
print('Max: ', Average1)
print("Min: ", Average1)
print('\n')
print('Experiment 02:')
print('Average: ', Average2)
print('Max: ', max(Ex02_times))
print("Min: ", min(Ex02_times))
print('\n')
print('Experiment 03:')
print('Average: ', Average3)
print('Max: ', max(Ex03_times))
print("Min: ", min(Ex03_times))"""

E3.EXPERIMENT03()
"""
Iterations_amount = 10
Ex05_noprio_times = [0]*Iterations_amount
Ex05_noprio_times_agent = [0]*Iterations_amount
Ex05_prio_times = [0]*Iterations_amount
Ex05_prio_times_agent = [0]*Iterations_amount
Average1_group = 0
Average2_group = 0
Average1_agent = 0
Average2_agent = 0
c_time = time.time()
print(Ex05_noprio_times_agent)
for index in range(Iterations_amount):
    (Ex05_noprio_times[index], Ex05_noprio_times_agent[index]) = E5.EXPERIMENT05(0, False)
    print(Ex05_noprio_times_agent[index])
    print(Ex05_noprio_times[index])
    (Ex05_prio_times[index], Ex05_prio_times_agent[index]) = E5.EXPERIMENT05(0, True)
    print(Ex05_prio_times_agent[index])
    print(Ex05_prio_times[index])
    Average1_group = Average1_group + round((Ex05_noprio_times[index]/Iterations_amount), 4)
    Average1_agent = Average1_agent + round((Ex05_noprio_times_agent[index] / Iterations_amount), 4)
    Average2_group = Average2_group + round((Ex05_prio_times[index] / Iterations_amount), 4)
    Average2_agent = Average2_agent + round((Ex05_prio_times_agent[index] / Iterations_amount), 4)
    Finished = (index+1)/Iterations_amount
    print('\n')
    print('Finished [%]: ', 100*Finished, '%')
    c_time2 = time.time()
    print('Estimated remaining time (hours): ', (c_time2-c_time)*(-1+1/Finished)/3600)
    print('\n')

print('Array1: ', Ex05_noprio_times)
print('Array2: ', Ex05_noprio_times_agent)
print('Experiment with no prio:')
print('Average(group): ', Average1_group)
print('Max(group): ', max(Ex05_noprio_times))
print("Min (group): ", min(Ex05_noprio_times))

print('Average(agent): ', Average1_agent)
print('Max(agent): ', max(Ex05_noprio_times_agent))
print("Min (agent): ", min(Ex05_noprio_times_agent))

print('\n')

print('Experiment with prio:')
print('Average(group): ', Average2_group)
print('Max(group): ', max(Ex05_prio_times))
print("Min (group): ", min(Ex05_prio_times))

print('Average(agent): ', Average2_agent)
print('Max(agent): ', max(Ex05_prio_times_agent))
print("Min (agent): ", min(Ex05_prio_times_agent))

print('\n')

""""""
Iterations_amount = 30
Ex03_times = [0]*Iterations_amount
Average3 = 0
c_time = time.time()
for index in range(Iterations_amount):
    Ex03_times[index] = E3.EXPERIMENT03()
    print(Ex03_times[index])
    Average3 = Average3 + round((Ex03_times[index]/Iterations_amount),4)
    Finished = (index+1)/Iterations_amount
    print('Finished [%]: ', 100*Finished, '%')
    c_time2 = time.time()
    print('Estimated remaining time: ', (c_time2-c_time)*(-1+1/Finished))

print('Experiment 01:')
print('Average: ', Average1)
print('Max: ', Average1)
print("Min: ", Average1)
print('\n')
print('Experiment 02:')
print('Average: ', Average2)
print('Max: ', max(Ex02_times))
print("Min: ", min(Ex02_times))
print('\n')
print('Experiment 03:')
print('Average: ', Average3)
print('Max: ', max(Ex03_times))
print("Min: ", min(Ex03_times))"""