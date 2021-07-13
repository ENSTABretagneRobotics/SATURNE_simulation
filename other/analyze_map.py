import numpy as np
import matplotlib.pyplot as plt
import pyproj
from mpl_toolkits.mplot3d import Axes3D

def spike(min_value,max_value,min_id,max_id):
    if((max_value-min_value) > huge_spike_detect and abs(min_id-max_id)<huge_spike_detect_nb):
        return [True,True,abs(max_value-min_value)]
    elif(max_value-min_value > small_spike_detect and abs(min_id-max_id)<small_spike_detect_nb):
        return [True,False,abs(max_value-min_value)]   
    else:
        return [False,False,0]
    
    

#files = ["magmap_v1.csv","magmap_v5.csv"]
#files = ["log_data.1611839109.022858.csv","log_data.1612471641.013366.csv"]
files = ["log_data.1612471641.013366.csv"]
proj = pyproj.Proj(proj='utm', zone="30U", ellps='WGS84')
huges_spikes_list = []
liste_X, liste_Y = [],[]
for file in files:
    txt = np.genfromtxt((x for x in open(file)),delimiter=';')
    
    X,Y,N = [],[],[]
    Xt,Yt,Nt = [],[],[]
    T = []
    
    txt[-1,0] = 0
    for i in range(len(txt[:,1])-1):
        if(txt[i,0] == 0):
            j = 1
            p = proj(txt[i,1],txt[i,2])
            x0,y0 = p[0],p[1] 
            while(txt[i+j,0] != 0):
                j = j + 1
            

            p = proj(txt[i+j,1],txt[i+j,2])
            #if(y0 > -794200):
                #print(txt[i+j,1],txt[i+j,2])
            x1,y1 = p[0],p[1]
            for k in range(j):
                X.append(x0+k*(x1-x0)/j)
                Y.append(y0+k*(y1-y0)/j)
                N.append(txt[i+k,3])
                T.append(len(N))
                if(txt[i+k,3] > 5.3*10**(-5)):
                    print(txt[i+j,0],txt[i+j,1],txt[i+j,2])

       
    print("\n")         
    X = -np.array(X)
    Y = np.array(Y)
    N = np.array(N)
    T = np.array(T)
    



    plt.figure()
       
    plt.plot(T,N,"x")
    
    
    last_min = N[0]
    last_max = N[0]
    last_min_id = 0
    last_max_id = 0
    already_used_min = False
    already_used_max = False
    last_value = N[0]
    
    up_mode = True
    
    
    taux_variation_max = 0.001*10**(-5)
    
    huge_spike_detect = 0.05*10**(-5)
    huge_spike_detect_nb = 150
    small_spike_detect = 0.02*10**(-5)
    small_spike_detect_nb = 75
    
    X_spike_huge = []
    Y_spike_huge = []
    N_spike_huge = []
    X_spike_small = []
    Y_spike_small = []
    N_spike_small = []    
    
    for i in range(1,len(N)):
        if(up_mode):
            taux_variation = N[i]-last_max
            if(taux_variation > 0 ):
                last_max = N[i]
                last_max_id = i
            elif(taux_variation < 0 and abs(taux_variation) > taux_variation_max):
                up_mode = False
                already_used_max = False
                #plt.plot([last_max_id],[last_max],"oy")
                if(already_used_max == False and already_used_min == False):
                    spk =  spike(last_min,last_max,last_min_id,last_max_id)
                    if(spk[0]):
                        if(spk[1]):
                            plt.plot([last_min_id,last_max_id],[last_min,last_max],"r")
                            X_spike_huge.append(X[int((last_min_id+last_max_id)/2)])
                            Y_spike_huge.append(Y[int((last_min_id+last_max_id)/2)])
                            N_spike_huge.append(spk[2])
                        else:
                            plt.plot([last_min_id,last_max_id],[last_min,last_max],"g")
                            X_spike_small.append(X[int((last_min_id+last_max_id)/2)])
                            Y_spike_small.append(Y[int((last_min_id+last_max_id)/2)])
                            N_spike_small.append(spk[2])
    
                        already_used_max = True
                        already_used_min = True
                last_min = N[i]
                last_min_id = i
                
        else:
            taux_variation = N[i]-last_min
            if(taux_variation < 0 ):
                last_min = N[i]
                last_min_id = i
            elif(taux_variation > 0 and abs(taux_variation) > taux_variation_max):
                up_mode = True
                already_used_min = False
                #plt.plot([last_min_id],[last_min],"oy")
                if(already_used_max == False and already_used_min == False):
                    spk =  spike(last_min,last_max,last_min_id,last_max_id)
                    if(spk[0]):
                        if(spk[1]):
                            plt.plot([last_min_id,last_max_id],[last_min,last_max],"r")
                            X_spike_huge.append(X[int((last_min_id+last_max_id)/2)])
                            Y_spike_huge.append(Y[int((last_min_id+last_max_id)/2)])
                            N_spike_huge.append(spk[2])
                        else:
                            plt.plot([last_min_id,last_max_id],[last_min,last_max],"g")
                            X_spike_small.append(X[int((last_min_id+last_max_id)/2)])
                            Y_spike_small.append(Y[int((last_min_id+last_max_id)/2)])
                            N_spike_small.append(spk[2])
                        already_used_max = True
                        already_used_min = True
                last_max = N[i]
                last_max_id = i
                
                
        last_value = N[i]
    liste_X.append(X)
    liste_Y.append(Y)
    huges_spikes_list.append([np.array(X_spike_huge),np.array(Y_spike_huge),np.array(N_spike_huge)])
    """
    print("Spikes : {}".format(len(X_spike_huge)+len(X_spike_small)))
    plt.figure()
       
    ##plt.plot(X,Y,"x")
    plt.plot(X_spike_huge,Y_spike_huge,"or",label="Large spikes")
    plt.plot(X_spike_small,Y_spike_small,"oy",label="Small spikes")
    plt.legend()
    
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    n = 100
    
    ax.scatter(X, Y, N,c=N)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Mag')
    """

#np.savetxt('test.txt', np.array([X,Y,N]).T, delimiter=' ') 
plt.figure()
for i in range(0,len(files)):
    plt.plot(liste_X[i],liste_Y[i],"b,")
    plt.scatter(huges_spikes_list[i][0],huges_spikes_list[i][1],c=(huges_spikes_list[i][2]))
plt.xlabel("X (in m)")
plt.ylabel("Y (in m)")
cbar = plt.colorbar()
cbar.set_label('Anomaly (in mT)')
plt.show()
