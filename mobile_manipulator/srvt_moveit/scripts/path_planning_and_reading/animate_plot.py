import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

x_vals = []
y_vals = []
z_vals = []

class AnimationPlot:

    def __init__(self, xline, yline, zline, line_count):
        self.xline = xline
        self.yline = yline
        self.zline = zline
        self.position_val = line_count

    def live_lister(self, count):
        x = float(self.xline[count])
        y = float(self.yline[count])
        z = float(self.zline[count])
        return x,y,z

    def animation_function(self):
        i = 0
        def animate(i):
            ax = plt.axes(projection='3d')

            x, y, z = self.live_lister(i)
            x_vals.append(x)
            y_vals.append(y)
            z_vals.append(z)

            ax.plot3D(x_vals, y_vals, z_vals, 'red', label='trajectory')
            print(i)
            plt.legend(loc='upper left')
            plt.tight_layout()
        
        while i <= self.position_val:
            ani = FuncAnimation(plt.gcf(), animate, interval=1000)
            plt.tight_layout()
            plt.show()
            i+=1





