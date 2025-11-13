import dimod
import dwave_networkx as dnx
import networkx as nx
import dwave.embedding
from dwave.system import LeapHybridCQMSampler 

import sys
from itertools import groupby


def read_input():
    input_file = open(sys.argv[1], 'r')

    n = int(input_file.readline().strip())
    weight_matrix = [[None for i in range(n)] for y in range(n)]

    for i in range(n):
        weights = input_file.readline().split()
        for j in range(n):
            weight_matrix[i][j] = int(weights[j].strip())

    input_file.close()

    return n,weight_matrix

def print_constraint(model):
    for const in model.constraints:
        
        print(model.constraints[const].to_polystring())

def build_model(n, weight_matrix, m, T):
    model = dimod.ConstrainedQuadraticModel()
    #var_list = dimod.variables.Variables(['x_' + str(i) + '_' + str(j) for i in range(1,n) for j in range(1,n)])

    var_list = []
    for i in range(m):
        for v in range(1,n):
            for t in range(1, T+1):
                var_label = 'x_' + str(i) + '_' + str(v) + '_' + str(t)
                var_list.append(var_label)
    #print(var_list)
      
    model.add_variables('BINARY', var_list)
    #print(model.variables)

    # all drones start at vertex 0
    var_list = []
    for i in range(m):
        var_label = 'x_' + str(i) + '_0_' + '0'
        var_list.append(var_label)
    model.add_variables('BINARY', var_list)
    #print(var_list) 
    #for var in var_list:
    #    model.fix_variable(var, 1)
    #print(model.variables)

    # compute NR(v,t) sets
    NR = {} 

    # set NR(0,0) 
    unreachable_pairs = []
    for delta in range(1,T+1):
        for u in range(1,n):      
            if weight_matrix[0][u] > delta:
                unreachable_pairs.append((u,delta))
    NR[0,0] = unreachable_pairs
    #print(NR[0,0])
    #print(NR)
    for v in range(1,n):
        for t in range(1, T):
            unreachable_pairs = []
            for delta in range(1, T-t+1):
                for u in range(1,n):
                    if not(u == v) and weight_matrix[v][u] > delta:
                            unreachable_pairs.append((u,delta))
            NR[v,t] = unreachable_pairs

    #print(NR[3,8])

    # set up constraints
    # P1
    for v in range(1, n):
        terms = []
        for i in range(m):
            for t in range(1, T+1):
                terms.append(['x_' + str(i) + '_' + str(v) + '_' + str(t),1])
        # vertex v must be visited once is labeled as P1_v
        model.add_constraint_from_iterable(terms, '==', rhs = 1, label = 'P1_' + str(v))

    #print(model.constraints['P1_2'].to_polystring())

    
    # P2
    for i in range(m):
        for t in range(1, T+1):
            terms = []

            for v in range(1,n):
                terms.append(['x_' + str(i) + '_' + str(v) + '_' + str(t),1])
        # drone i visits at most 1 vertex at time t is labeled as P2_i_t
            label = 'P2_' + str(i) + '_' + str(t)
            #print(label)
            model.add_constraint_from_iterable(terms, '<=', rhs = 1, label = label)
        
    #print(model.constraints['P2_2_4'].to_polystring())
     
    # P3
    # first step is encoded explicitly

    for i in range(m):
        for (u, delta) in NR[0,0]:
            terms = []
            terms.append(['x_' + str(i) + '_0_' + '0',1])
            terms.append(['x_' + str(i) + '_' + str(u) + '_' + str(delta),1])
            label = 'x_' + str(i) + '_0_' + '0*x_' + str(i) + '_' + str(u) + '_' + str(delta) 
            model.add_constraint_from_iterable(terms, '<=', rhs = 1,label=label)
    #print(model.constraints['x_1_0_0*x_1_2_7']) 

    # encode the unreachable constraints
    # I had to change this implementation since the hybrid solver has a limit on the 
    # number of constraints, the new implementation below groups constraints together to
    # reduce their number (so it replaces a large number of simple constraints with a less
    # number of more complicated ones)

    #for i in range(m):
    #    for v in range(1,n):
    #        for t in range(1,T):
    #            for (u,delta) in NR[v,t]:
    #                terms = []
    #                terms.append(['x_' + str(i) + '_' + str(v) + '_' + str(t),1])
    #                terms.append(['x_' + str(i) + '_' + str(u) + '_' + str(t+delta),1])
    #                label = 'x_' + str(i) + '_' + str(v) + '_' + str(t) + '*x_' + str(i) + '_' + str(u) + '_' + str(t+delta)
    #                model.add_constraint_from_iterable(terms, '<=', rhs = 1, label=label)


    # encode the unreachable constraints differently
    
    for i in range(m):
        for v in range(1,n):
            for t in range(1,T):
                terms = []
                x_1 = 'x_' + str(i) + '_' + str(v) + '_' + str(t)
                for (u,delta) in NR[v,t]:
                    x_2 = 'x_' + str(i) + '_' + str(u) + '_' + str(t+delta)
                    terms.append((x_1,x_2,1))
                    label = 'x_' + str(i) + '_' + str(v) + '_' + str(t)
                model.add_constraint_from_iterable(terms, '==', rhs = 0, label=label)
    #print(model.constraints['x_0_4_8'].to_polystring()) 
    #print_constraint(model)
    
    # fix x_i_0_0 to enforce all drones to start at vertex 0
    for i in range(m):
        var = 'x_' + str(i) + '_0_0'
        model.fix_variable(var, 1)

    obj = []
    

    model.set_objective(obj)
    #print(model.objective.to_polystring())
    #print(model.constraints)

    #    f += q[v,1] * weight_matrix[0][v]
    #print(f)
    #for t in range(1, n-1):
    #    for v in range(1,n):
    #        for v_p in range(1,n):
    #            if not(v == v_p):
    #                f += weight_matrix[v][v_p]*q[v,t]*q[v_p,t+1]
    #print(f)
    #model = f


    return model

def run_cqm_and_collect_solutions(model, sampler):
    sampleset = sampler.sample_cqm(model, time)
    return sampleset

def build_path(solution, n, k):
    #print(solution)
    #print('0')
    path = [0] 
    for v in range(1,n):
        if solution['x_' + str(v) + '_' + str(1)] == 1:
            #print('Step 1:' + str(v))
            path.append(v)

    for t in range(2,k):
        for v in range(n):
            if solution['x_' + str(v) + '_' + str(t)] == 1:
                #print('Step ' + str(t) + ': ' + str(v))
                path.append(v)
                

    return path

def process_solutions(sampleset, n, m, T, weight_matrix):
    feasible_solutions = []
    
    #print(sampleset.record)
    for solution in sampleset:
        #print(solution)
        if(model.check_feasible(solution, atol = 0)):
        #if check_feasibility(solution, n, k) == True:
            feasible_solutions.append(solution)
    makespans = []   
    for solution in feasible_solutions:
        makespan, routes = check_feasibility(solution, n, m, T, weight_matrix)
        makespans.append((makespan, routes))

  
     
    # return best makespan found
    # 1000 is just an arbitrarily large default
    best_makespan = 1000
    best_routes = []
    for (makespan, routes) in makespans:
        if makespan < best_makespan:
            best_makespan = makespan
            best_routes = routes
    return best_makespan, best_routes 
      

def check_feasibility(solution, n, m, T, weight_matrix):
   
    #print(solution)
    

    # constraint 1 - all vertices are visited
    vertices_visited = []
    for v in range(1, n):
        for i in range(m):
            for t in range(1, T+1):
                var_label = 'x_' + str(i) + '_' + str(v) + '_' + str(t)
                # check if v is visited
                if solution[var_label] == 1:
                    if v not in vertices_visited:
                        vertices_visited.append(v)
                    else:
                        print(v, ' is visited more than once')
                        print(solution)
                        sys.exit()
    if not(len(vertices_visited) == (n-1)):
        vertices_visited.sort()
        print(vertices_visited)
        print('Not all veritces were visited')
        print(solution)
        sys.exit()

    # constraint 2 - each drone visit at most 1 vertex at a time
    for i in range(m):
        for t in range(1, T+1):
            x_sum = 0
            for v in range(1, n):
                var_label = 'x_' + str(i) + '_' + str(v) + '_' + str(t)
                x_sum += solution[var_label]
            if x_sum > 1:
                print('Drone ', i, ' visits more than one vertex at time ', t)
                print(solution)
                sys.exit()

    
    # constraint 3 - reachability constraints
    routes = []
    for i in range(m):
        drone_route = [(0,0)]
        for t in range(1, T+1):
            for v in range(1, n):
                var_label = 'x_' + str(i) + '_' + str(v) + '_' + str(t)
                if solution[var_label] == 1:
                    drone_route.append((v,t))

        routes.append(drone_route)
    
    for i in range(m):
        route = routes[i]
   
        for j in range(1, len(route)-1):
            u = route[j][0]
            v = route[j+1][0]
            t_1 = route[j][1]
            t_2 = route[j+1][1]
            
            if (t_2 - t_1) < weight_matrix[u][v]:
                    
                print('Reachibility error with drone ', i, (u,t_1), (v, t_2))
                print(t_2-t_1)
                print(weight_matrix[u][v])
                print(solution)
                sys.exit()
    times = []
    temp_routes = []
    
    
    # remove 'gaps' inbetween travel times
    for i in range(m):
        time = 0
        route = routes[i]
        temp_route = [0]
        for j in range(len(route)-1):
            u = route[j][0]
            v = route[j+1][0]
            time += weight_matrix[u][v]
            temp_route.append(route[j+1][0])
        times.append(time)
        temp_routes.append(temp_route)


    makespan = max(times)
    return makespan, temp_routes
    
# set time limit
time = 15

# set default value for m
m = 3
if '-m' in sys.argv:
    m = int(sys.argv[sys.argv.index('-m')+1].strip()) 

# set default value for T
T = 20
if '-T' in sys.argv:
    m = int(sys.argv[sys.argv.index('-T')+1].strip()) 

# replace with your D-Wave API token
token = ''


n, weight_matrix = read_input()

model = build_model(n, weight_matrix, m, T)

sampler = LeapHybridCQMSampler()
#print(sampler.properties)

sampleset = run_cqm_and_collect_solutions(model, sampler)
best_makespan, routes = process_solutions(sampleset, n, m, T, weight_matrix)

print(best_makespan)
for route in routes:
    print(route)





