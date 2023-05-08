from realtime_rrt import basic_rrt
from realtime_rrt_sliding_window import ours
import matplotlib.pyplot as plt

def compare1(n):
    iter=[[],[]]
    node=[[],[]]
    time=[[],[]]
    for i in range(1,n+1):
        a,b,t1 = basic_rrt()
        print(f"basic:{i}")
        c,d,t2 = ours()
        print(f"ours:{i}")
        
        iter[0].append(a)
        node[0].append(b)
        iter[1].append(c)
        node[1].append(d)
        time[0].append(t1)
        time[1].append(t2)
        
    print(iter)
    print(node)
    print(f"basic time avg:{sum(time[0])/len(time[0])}")
    print(f"ours time avg:{sum(time[1])/len(time[1])}")
    print(f"basic node avg:{sum(node[0])/len(node[0])}")
    print(f"ours node avg:{sum(node[1])/len(node[1])}")
    i=range(1,n+1)
    
    # 创建画布1和坐标轴对象1
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)

    # 绘制图1
    ax1.plot(i, time[0])
    ax1.plot(i, time[1])
    ax1.set_title('Time')
    ax1.set_xlabel('X Axis 1')
    ax1.set_ylabel('Y Axis 1')

    # 创建画布2和坐标轴对象2
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)

    # 绘制图2
    plt.plot(i, node[0])
    plt.plot(i, node[1])
    ax2.set_title('Node')
    ax2.set_xlabel('X Axis 2')
    ax2.set_ylabel('Y Axis 2')

    # 显示图形
    plt.show()
    
def compare2(n):
    iter=[[],[]]
    node=[[],[]]
    time=[[],[]]
    for i in range(1,n+1):
        a,b,t1 = ours(straight=True)
        print(f"basic:{i}")
        c,d,t2 = ours()
        print(f"ours:{i}")
        
        iter[0].append(a)
        node[0].append(b)
        iter[1].append(c)
        node[1].append(d)
        time[0].append(t1)
        time[1].append(t2)
        
    print(iter)
    print(node)
    print(f"straight time avg:{sum(time[0])/len(time[0])}")
    print(f"ours time avg:{sum(time[1])/len(time[1])}")
    print(f"straight node avg:{sum(node[0])/len(node[0])}")
    print(f"ours node avg:{sum(node[1])/len(node[1])}")
    i=range(1,n+1)
    
    # 创建画布1和坐标轴对象1
    fig1 = plt.figure()
    ax1 = fig1.add_subplot(111)

    # 绘制图1
    ax1.plot(i, time[0])
    ax1.plot(i, time[1])
    ax1.set_title('Time')
    ax1.set_xlabel('X Axis 1')
    ax1.set_ylabel('Y Axis 1')

    # 创建画布2和坐标轴对象2
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)

    # 绘制图2
    plt.plot(i, node[0])
    plt.plot(i, node[1])
    ax2.set_title('Node')
    ax2.set_xlabel('X Axis 2')
    ax2.set_ylabel('Y Axis 2')

    # 显示图形
    plt.show()
    
    

if __name__ == '__main__':
    compare2(10)



