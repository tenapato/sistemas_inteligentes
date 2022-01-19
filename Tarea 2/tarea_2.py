import numpy as np
import time

class Node():
    def __init__(self,estado,padre,accion,profundidad,costo_paso,costo_ruta,costo_heuristica):
        self.estado = estado 
        self.padre = padre 
        self.accion = accion 
        self.profundidad = profundidad 
        self.costo_paso = costo_paso 
        self.costo_ruta = costo_ruta 
        self.costo_heuristica = costo_heuristica 

        # nodo hijo 
        self.mover_arriba = None 
        self.mover_izquierda = None
        self.mover_abajo = None
        self.mover_derecha = None
    
    # verifica si el movimiento es valido 

    def prueba_mover_abajo(self):
        indice_cero=[i[0] for i in np.where(self.estado==0)] 
        if indice_cero[0] == 0:
            return False
        else:
            valor_arriba = self.estado[indice_cero[0]-1,indice_cero[1]] 
            estado_nuevo = self.estado.copy()
            estado_nuevo[indice_cero[0],indice_cero[1]] = valor_arriba
            estado_nuevo[indice_cero[0]-1,indice_cero[1]] = 0
            return estado_nuevo,valor_arriba
        
    def prueba_mover_derecha(self):
        indice_cero=[i[0] for i in np.where(self.estado==0)] 
        if indice_cero[1] == 0:
            return False
        else:
            valor_izquierda = self.estado[indice_cero[0],indice_cero[1]-1] 
            estado_nuevo = self.estado.copy()
            estado_nuevo[indice_cero[0],indice_cero[1]] = valor_izquierda
            estado_nuevo[indice_cero[0],indice_cero[1]-1] = 0
            return estado_nuevo,valor_izquierda
        
    def prueba_valor_arriba(self):
        indice_cero=[i[0] for i in np.where(self.estado==0)] 
        if indice_cero[0] == 2:
            return False
        else:
            valor_abajo = self.estado[indice_cero[0]+1,indice_cero[1]] 
            estado_nuevo = self.estado.copy()
            estado_nuevo[indice_cero[0],indice_cero[1]] = valor_abajo
            estado_nuevo[indice_cero[0]+1,indice_cero[1]] = 0
            return estado_nuevo,valor_abajo
        
    def prueba_mover_izquierda(self):
        indice_cero=[i[0] for i in np.where(self.estado==0)] 
        if indice_cero[1] == 2:
            return False
        else:
            valor_derecha = self.estado[indice_cero[0],indice_cero[1]+1] #
            estado_nuevo = self.estado.copy()
            estado_nuevo[indice_cero[0],indice_cero[1]] = valor_derecha
            estado_nuevo[indice_cero[0],indice_cero[1]+1] = 0
            return estado_nuevo,valor_derecha
    
    # return user specified heuristic cost
    '''
    def get_h_costo(self,estado_nuevo,estado_final,heuristic_function,costo_ruta,profundidad):
        if heuristic_function == 'num_misplaced':
            return self.costo_fuera_h(estado_nuevo,goal_state)
        elif heuristic_function == 'manhattan':
            return self.h_manhattan_cost(estado_nuevo,goal_state)
        # since this game is made unfair by setting the step cost as the value of the tile being moved
        # to make it fair, I made all the step cost as 1
        # made it a best-first-search with manhattan heuristic function
        elif heuristic_function == 'fair_manhattan':
            return self.h_manhattan_cost(estado_nuevo,goal_state) - costo_ruta + profundidad
       ''' 
    # costo de la heuristica: nnumero de cuado fuera de lugar 
    def costo_fuera_h(self,estado_nuevo,estado_final):
        cost = np.sum(estado_nuevo != estado_final)-1 
        if cost > 0:
            return cost
        else:
            return 0 
    '''
    # return heuristic cost: sum of Manhattan distance to reach the goal estado
    def h_manhattan_cost(self,estado_nuevo,estado_final):
        current = estado_nuevo
        # digit and coordinates they are supposed to be
        goal_position_dic = {1:(0,0),2:(0,1),3:(0,2),8:(1,0),0:(1,1),4:(1,2),7:(2,0),6:(2,1),5:(2,2)} 
        sum_manhattan = 0
        for i in range(3):
            for j in range(3):
                if current[i,j] != 0:
                    sum_manhattan += sum(abs(a-b) for a,b in zip((i,j), goal_position_dic[current[i,j]]))
        return sum_manhattan
        '''
    # cuando encuentro el nodo final, se rastrea la ruta a el estado raiz e imprime 
    def imprimir_camino(self):
        
        rastro_estado = [self.estado]
        rastro_accion = [self.accion]
        rastro_profundidad = [self.profundidad]
        rastro_costo_paso = [self.costo_paso]
        rastro_costo_ruta = [self.costo_ruta]
        rastro_costo_heuristica = [self.costo_heuristica]
        
    
        while self.padre:
            self = self.padre

            rastro_estado.append(self.estado)
            rastro_accion.append(self.accion)
            rastro_profundidad.append(self.profundidad)
            rastro_costo_paso.append(self.costo_paso)
            rastro_costo_ruta.append(self.costo_ruta)
            rastro_costo_heuristica.append(self.costo_heuristica)

    
        contador_pasos = 0
        while rastro_estado:
            print ('pasos',contador_pasos)
            print (rastro_estado.pop())
            print ('accion=',rastro_accion.pop(),', profundidad=',str(rastro_profundidad.pop()),\
            ', costo paso=',str(rastro_costo_paso.pop()),', costo total=',\
            str(rastro_costo_ruta.pop() + rastro_costo_heuristica.pop()),'\n')
            
            contador_pasos += 1



# modelo de busqueda best first
    def best_first_search(self, estado_final):
        primero = time.time()
        
        queue = [(self,0)] 
        numero_nodos_pop = 0 
        longitud_maxima = 1 
        
        queue_profundidad = [(0,0)] 
        costo_paso_queue = [(0,0)] 
        visito = set([]) 
        
        while queue:
            # ordena en orden asecende segun su costo heuristoco 
            queue = sorted(queue, key=lambda x: x[1])
            queue_profundidad = sorted(queue_profundidad, key=lambda x: x[1])
            costo_paso_queue = sorted(costo_paso_queue, key=lambda x: x[1])
            
            if len(queue) > longitud_maxima:
                longitud_maxima = len(queue)
                
            nodo_actual = queue.pop(0)[0] 

            
            numero_nodos_pop += 1 
            profundidad_actual = queue_profundidad.pop(0)[0] 
            costo_camino_actual = costo_paso_queue.pop(0)[0] 
            visito.add(tuple(nodo_actual.estado.reshape(1,9)[0])) 
            
            # rastrea hasta la raiz
            if np.array_equal(nodo_actual.estado,estado_final):
                nodo_actual.imprimir_camino()
                
                print ('Rendimiento en tiempo:',str(numero_nodos_pop),'nodos pop')
                print ('Rendimiento del espacio:', str(longitud_maxima),'maximo nodos en la cola ')
                print ('Tiempo que se uso: %0.2fs' % (time.time()-primero))
                return True
    
            else:     
                # ve si el movimiento es valido
                if nodo_actual.prueba_mover_abajo():
                    estado_nuevo,valor_arriba = nodo_actual.prueba_mover_abajo()
                    if tuple(estado_nuevo.reshape(1,9)[0]) not in visito:
                        h_costo = self.costo_fuera_h(estado_nuevo,estado_final)
                        nodo_actual.mover_abajo = Node(estado=estado_nuevo,padre=nodo_actual,accion='abajo',profundidad=profundidad_actual+1,\
                                              costo_paso=valor_arriba,costo_ruta=costo_camino_actual+valor_arriba,costo_heuristica=h_costo)
                        queue.append((nodo_actual.mover_abajo,h_costo))
                        queue_profundidad.append((profundidad_actual+1,h_costo))
                        costo_paso_queue.append((costo_camino_actual+valor_arriba,h_costo))
                    
                 # ve si el movimiento es valido
                if nodo_actual.prueba_mover_derecha():
                    estado_nuevo,valor_izquierda = nodo_actual.prueba_mover_derecha()
                    if tuple(estado_nuevo.reshape(1,9)[0]) not in visito:
                        h_costo = self.costo_fuera_h(estado_nuevo,estado_final)
                        nodo_actual.mover_derecha = Node(estado=estado_nuevo,padre=nodo_actual,accion='derecha',profundidad=profundidad_actual+1,\
                                              costo_paso=valor_izquierda,costo_ruta=costo_camino_actual+valor_izquierda,costo_heuristica=h_costo)
                        queue.append((nodo_actual.mover_derecha,h_costo))
                        queue_profundidad.append((profundidad_actual+1,h_costo))
                        costo_paso_queue.append((costo_camino_actual+valor_izquierda,h_costo))
                    
                 # ve si el movimiento es valido
                if nodo_actual.prueba_valor_arriba():
                    estado_nuevo,valor_abajo = nodo_actual.prueba_valor_arriba()
                    if tuple(estado_nuevo.reshape(1,9)[0]) not in visito:
                        h_costo = self.costo_fuera_h(estado_nuevo,estado_final)
                        nodo_actual.mover_arriba = Node(estado=estado_nuevo,padre=nodo_actual,accion='arriba',profundidad=profundidad_actual+1,\
                                              costo_paso=valor_abajo,costo_ruta=costo_camino_actual+valor_abajo,costo_heuristica=h_costo)
                        queue.append((nodo_actual.mover_arriba,h_costo))
                        queue_profundidad.append((profundidad_actual+1,h_costo))
                        costo_paso_queue.append((costo_camino_actual+valor_abajo,h_costo))

                # ve si el movimiento es valido
                if nodo_actual.prueba_mover_izquierda():
                    estado_nuevo,valor_derecha = nodo_actual.prueba_mover_izquierda()
                    if tuple(estado_nuevo.reshape(1,9)[0]) not in visito:
                        h_costo = self.costo_fuera_h(estado_nuevo,estado_final)
                        nodo_actual.mover_izquierda = Node(estado=estado_nuevo,padre=nodo_actual,accion='izquierda',profundidad=profundidad_actual+1,\
                                              costo_paso=valor_derecha,costo_ruta=costo_camino_actual+valor_derecha,costo_heuristica=h_costo)
                        queue.append((nodo_actual.mover_izquierda,h_costo))
                        queue_profundidad.append((profundidad_actual+1,h_costo))
                        costo_paso_queue.append((costo_camino_actual+valor_derecha,h_costo))


#test = np.array([1,2,3,8,6,4,7,5,0]).reshape(3,3)
#easy = np.array([1,3,4,8,6,2,7,0,5]).reshape(3,3)
#medium = np.array([2,8,1,0,4,3,7,6,5]).reshape(3,3)
prueba1 = np.array([5,6,7,4,0,8,3,2,1]).reshape(3,3)

estado_inicial = prueba1
estado_final = np.array([1,2,3,4,5,6,7,8,0]).reshape(3,3)
print (estado_inicial,'\n')
print (estado_final)



root_node = Node(estado=estado_inicial,padre=None,accion=None,profundidad=0,costo_paso=0,costo_ruta=0,costo_heuristica=0)


# busqueda que se basa en el costo heuristica del numero de fuera de lugar, usando queue
root_node.best_first_search(estado_final)