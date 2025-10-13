from amplify_sched import *
import sys

def read_input():
    input_file = open(sys.argv[1],'r')
    n = int(input_file.readline().strip())
    weight_matrix = [[ None for i in range(n)] for y in range(n)]
    for i in range(n):
        weights = input_file.readline().split()
        for j in range(n):
            weight_matrix[i][j] = int(weights[j].strip())
    replaced_edges = eval(input_file.readline().strip())
    input_file.close()
    return n, weight_matrix,replaced_edges


n, weight_matrix,replaced_edges = read_input()

# m is the number of drones
m = 3
# define Model
model = Model()

# add the drones to model
for i in range(m):
    model.machines.add('Drone_' + str(i))


# Job 0 is a pseudo-job to enforce all drones start at v_0
model.jobs.add('v_0')
for i in range(m):
    # Job 0 has m tasks, each task can only be completed by one specific drone
    model.jobs['v_0'].append(Task())
    model.jobs['v_0'][i].processing_times['Drone_' + str(i)] = 0

# add 1 job for each vertex in the graph
for i in range(1,n):
    job_id = 'v_' + str(i)
    model.jobs.add(job_id)

    # each job has only 1 task
    model.jobs[job_id].append(Task())

    # other jobs cannot start until v_0 is done (force all drones to start at v_0)
    model.jobs[job_id].dependent_jobs.append('v_0')
    
    for j in range(m):
        drone_id = 'Drone_' + str(j)

        # each job can be done by any drone
        model.jobs[job_id][0].processing_times[drone_id] = 0

# for every drone, we add the distance between vertices u and v as setup time
for i in range(m):
    drone_id = 'Drone_' + str(i)
    for u in range(n):
        for v in range(n):
            job_id_1 = 'v_' + str(u)
            job_id_2 = 'v_' + str(v)
            distance = weight_matrix[u][v]

            # distance = 0 means v is not reachable from u
            if distance == 0:
                # set some arbitrary large time if this is the case
                time = 10000

            # process id for v_0 is different for each drone to make sure 
            # each drone does v_0 before moving on to other tasks
            process_id_1 = 0
            process_id_2 = 0
            if u == 0:
                process_id_1 = i
            if v == 0:
                process_id_2 = i
            model.machines[drone_id].setup_times.append((distance, model.jobs[job_id_1,process_id_1], model.jobs[job_id_2,process_id_2]))




# replace with your access token
token = ''

solution = model.solve(token=token, timeout=1)

# to see the timeline, you need to set a positive processing
# time for the tasks, otherwise the bar has width 0 and you cannot
# see them on the plot (note that this doesn't produce the correct
# result)
solution.timeline(machine_view=True).show()

table = solution.table



travel_routes = []
makespan = 0
# get sequence for each drone
for i in range(m):
    drone_id = 'Drone_' + str(i)
    seq = {}
    for index, row in table.iterrows():
        if row['Machine'] == drone_id:
            seq[row['Start']] = row['Job']
    
    time_list = list(seq.keys())
    time_list.sort()
    makespan = max(makespan,max(time_list))
    temp = []
    for t in time_list:
        temp.append(int(seq[t].split('_')[1]))
    travel_routes.append(temp)

# table contains the full information
#print(table)

print(makespan)
for route in travel_routes:
    print(route)
