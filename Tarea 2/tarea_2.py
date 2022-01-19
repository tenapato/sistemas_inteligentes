import numpy as np
import time

class Node():
    def __init__(self,estado,nodo_padre,accion,profundidad,costo_paso,costo_camino,costo_heuristica):
        self.estado = estado 
        self.nodo_padre = nodo_padre 
        self.accion = accion 
        self.profundidad = profundidad 
        self.costo_paso = costo_paso 
        self.costo_camino = costo_camino 
        self.costo_heuristica = costo_heuristica 
        
        # nodo hijo 
        self.arriba = None 
        self.izquierda = None
        self.abajo = None
        self.derecha = None

    
    # verifica movimientos

    def v_abajo(self):
        inicio=[i[0] for i in np.where(self.state==0)] 
        if inicio[0] == 0:
            return False
        else:
            up_value = self.state[inicio[0]-1,inicio[1]] 
            new_state = self.state.copy()
            new_state[inicio[0],inicio[1]] = up_value
            new_state[inicio[0]-1,inicio[1]] = 0
            return new_state,up_value
        
    def v_derecha(self):
        inicio=[i[0] for i in np.where(self.state==0)] 
        if inicio[1] == 0:
            return False
        else:
            left_value = self.state[inicio[0],inicio[1]-1] 
            new_state = self.state.copy()
            new_state[inicio[0],inicio[1]] = left_value
            new_state[inicio[0],inicio[1]-1] = 0
            return new_state,left_value
        
    def v_arriba(self):
        inicio=[i[0] for i in np.where(self.state==0)] 
        if inicio[0] == 2:
            return False
        else:
            lower_value = self.state[inicio[0]+1,inicio[1]] 
            new_state = self.state.copy()
            new_state[inicio[0],inicio[1]] = lower_value
            new_state[inicio[0]+1,inicio[1]] = 0
            return new_state,lower_value
        

    def v_izquierda(self):
        inicio=[i[0] for i in np.where(self.state==0)] 
        if inicio[1] == 2:
            return False
        else:
            right_value = self.state[inicio[0],inicio[1]+1] 
            new_state = self.state.copy()
            new_state[inicio[0],inicio[1]] = right_value
            new_state[inicio[0],inicio[1]+1] = 0
            return new_state,right_value
    
    # costo 
    def get_h_cost(self,new_state,goal_state,heuristic_function,path_cost,depth):
        if heuristic_function == 'num_misplaced':
            return self.h_misplaced_cost(new_state,goal_state)
        elif heuristic_function == 'manhattan':
            return self.h_manhattan_cost(new_state,goal_state)
        elif heuristic_function == 'fair_manhattan':
            return self.h_manhattan_cost(new_state,goal_state) - path_cost + depth
    
    # return heuristic cost: number of misplaced tiles
    def h_misplaced_cost(self,new_state,goal_state):
        cost = np.sum(new_state != goal_state)-1 # minus 1 to exclude the empty tile
        if cost > 0:
            return cost
        else:
            return 0 # when all tiles matches
    
    # return heuristic cost: sum of Manhattan distance to reach the goal state
    def h_manhattan_cost(self,new_state,goal_state):
        current = new_state
        # digit and coordinates they are supposed to be
        goal_position_dic = {1:(0,0),2:(0,1),3:(0,2),8:(1,0),0:(1,1),4:(1,2),7:(2,0),6:(2,1),5:(2,2)} 
        sum_manhattan = 0
        for i in range(3):
            for j in range(3):
                if current[i,j] != 0:
                    sum_manhattan += sum(abs(a-b) for a,b in zip((i,j), goal_position_dic[current[i,j]]))
        return sum_manhattan
        
    # once the goal node is found, trace back to the root node and print out the path
    def print_path(self):
        state_trace = [self.state]
        action_trace = [self.action]
        depth_trace = [self.depth]
        step_cost_trace = [self.step_cost]
        path_cost_trace = [self.path_cost]
        heuristic_cost_trace = [self.heuristic_cost]
        
        # add node information as tracing back up the tree
        while self.parent:
            self = self.parent

            state_trace.append(self.state)
            action_trace.append(self.action)
            depth_trace.append(self.depth)
            step_cost_trace.append(self.step_cost)
            path_cost_trace.append(self.path_cost)
            heuristic_cost_trace.append(self.heuristic_cost)

        # print out the path
        step_counter = 0
        while state_trace:
            print('step',step_counter)
            print(state_trace.pop())
            print('action=',action_trace.pop(),', depth=',str(depth_trace.pop()),\
            ', step cost=',str(step_cost_trace.pop()),', total_cost=',\
            str(path_cost_trace.pop() + heuristic_cost_trace.pop()),'\n')
            
            step_counter += 1
                    
    
                     
    # Algoritmo de busqueda utilizado
    def best_first_search(self, estado_meta):
        start = time.time()
        
        queue = [(self,0)] # cola de nodos no visitados
        queue_num_nodes_popped = 0 
        queue_max_length = 1 # numero maximo de nodos en la cola
        
        depth_queue = [(0,0)] # (profundidad, costo heuristico)
        path_cost_queue = [(0,0)] # (costo de camino, costo hueristico)
        visited = set([]) # estados visitados
        
        while queue:
            # ordena la cola basado en el costo heuristico en orden ascendente
            queue = sorted(queue, key=lambda x: x[1])
            depth_queue = sorted(depth_queue, key=lambda x: x[1])
            path_cost_queue = sorted(path_cost_queue, key=lambda x: x[1])
            
            # modificar el tamaño maximo de la cola
            if len(queue) > queue_max_length:
                queue_max_length = len(queue)
                
            current_node = queue.pop(0)[0] # sacar el primer nodo de la cola
            
            
            queue_num_nodes_popped += 1 
            current_depth = depth_queue.pop(0)[0] 
            current_path_cost = path_cost_queue.pop(0)[0] 
            visited.add(tuple(current_node.state.reshape(1,9)[0])) # agregar el estado actual a la lista de visitados
            
    
            # cuando el estado meta se encuentra, rastrear hacia el nodo raiz y imprimir el camino
            if np.array_equal(current_node.state,estado_meta):
                current_node.print_path()
                
                print ('Time performance:',str(queue_num_nodes_popped),'nodes popped off the queue.')
                print ('Space performance:', str(queue_max_length),'nodes in the queue at its max.')
                print ('Time spent: %0.2fs' % (time.time()-start))
                return True
            
            else:     
                # ver si mover hacia abajo es valido
                if current_node.try_move_down():
                    new_state,up_value = current_node.try_move_down()
                    # checar si el nodo resultante ya fue visitado
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        # obtener el costo heuristico
                        h_cost = self.h_misplaced_cost(new_state,estado_meta)
                        # crear un nuevo nodo hijo
                        current_node.move_down = Node(state=new_state,parent=current_node,action='down',depth=current_depth+1,\
                                              step_cost=up_value,path_cost=current_path_cost+up_value,heuristic_cost=h_cost)
                        queue.append((current_node.move_down,h_cost))
                        depth_queue.append((current_depth+1,h_cost))
                        path_cost_queue.append((current_path_cost+up_value,h_cost))
                    
                
                # # ver si mover hacia la derecha es valido
                if current_node.try_move_right():
                    new_state,left_value = current_node.try_move_right()
                    # checar si el nodo resultante ya fue visitado
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                       # obtener el costo heuristico
                        h_cost = self.h_misplaced_cost(new_state,estado_meta)
                        # crear un nuevo nodo hijo
                        current_node.move_right = Node(state=new_state,parent=current_node,action='right',depth=current_depth+1,\
                                              step_cost=left_value,path_cost=current_path_cost+left_value,heuristic_cost=h_cost)
                        queue.append((current_node.move_right,h_cost))
                        depth_queue.append((current_depth+1,h_cost))
                        path_cost_queue.append((current_path_cost+left_value,h_cost))
                    
                 # ver si mover hacia arriba es valido
                if current_node.try_move_up():
                    new_state,lower_value = current_node.try_move_up()
                    
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        
                        h_cost = self.h_misplaced_cost(new_state,estado_meta)
                        
                        current_node.move_up = Node(state=new_state,parent=current_node,action='up',depth=current_depth+1,\
                                              step_cost=lower_value,path_cost=current_path_cost+lower_value,heuristic_cost=h_cost)
                        queue.append((current_node.move_up,h_cost))
                        depth_queue.append((current_depth+1,h_cost))
                        path_cost_queue.append((current_path_cost+lower_value,h_cost))

                 # ver si mover hacia la izquierda es valido
                if current_node.try_move_left():
                    new_state,right_value = current_node.try_move_left()
                    
                    if tuple(new_state.reshape(1,9)[0]) not in visited:
                        
                        h_cost = self.h_misplaced_cost(new_state,estado_meta)
                        
                        current_node.move_left = Node(state=new_state,parent=current_node,action='left',depth=current_depth+1,\
                                              step_cost=right_value,path_cost=current_path_cost+right_value,heuristic_cost=h_cost)
                        queue.append((current_node.move_left,h_cost))
                        depth_queue.append((current_depth+1,h_cost))
                        path_cost_queue.append((current_path_cost+right_value,h_cost))

        test = np.array([1,2,3,8,6,4,7,5,0]).reshape(3,3)
        easy = np.array([1,3,4,8,6,2,7,0,5]).reshape(3,3)
        medium = np.array([2,8,1,0,4,3,7,6,5]).reshape(3,3)
        hard = np.array([5,6,7,4,0,8,3,2,1]).reshape(3,3)

        estado_inicial = easy
        estado_meta = np.array([1,2,3,8,0,4,7,6,5]).reshape(3,3)
        print(" Estado inicial: ",'\n' + str( estado_inicial),'\n')
        print(" Estado meta: ",'\n' + str( estado_meta),'\n')
        
        
        nodo_raiz = Node(estado=estado_inicial,nodo_padre=None,accion=None,profundidad=0,costo_paso=0,costo_camino=0,costo_heuristica=0)
                
        nodo_raiz.best_first_search(estado_meta)