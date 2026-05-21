from collections import defaultdict
import os
import re
import glob
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

LOG_DIR = "src/logs"

time_data = []
s_data = []

def find_latest_file():
    files = glob.glob(os.path.join(LOG_DIR, "*"))
    if not files:
        return None
    return max(files, key=os.path.getmtime)

log_file = find_latest_file()

if log_file is None:
    print("No log files found")
    exit()

print("Using log file:", log_file)

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(show=True)
win.setWindowTitle("Realtime log plot")

def create_plot(title, row, col, x_label, y_label, x_units, y_units):
    plot = win.addPlot(title=title, row=row, col=col)
    plot.setLabel('left',  text=y_label, units=y_units)
    plot.setLabel('bottom', text=x_label, units=x_units)
    plot.addLegend(labelTextColor='k')
    plot.getViewBox().setBackgroundColor('w')
    plot.showGrid(x=True, y=True, alpha=0.5)
    return plot

plot_01 = create_plot("Air, NTC, Setpoint", 0, 0, "time", "values", "mins", None)
plot_02 = create_plot("PID", 0, 1, "time", "values", "mins", None)
plot_03 = create_plot("Errors", 1, 0, "time", "values", "mins", None)
plot_13 = create_plot("Errors", 1, 1, "time", "values", "mins", None)

curve_air_temp = plot_01.plot(pen=pg.mkPen('r', width=3), name="Air temp")
curve_setpoint = plot_01.plot(pen=pg.mkPen('k', width=3), name="Setpoint")
curve_heater_temp = plot_01.plot(pen=pg.mkPen('b', width=3), name="Heater temp")

curve_pid_p_term = plot_02.plot(pen=pg.mkPen('r', width=3), name="pTerm")
curve_pid_i_term = plot_02.plot(pen=pg.mkPen('b', width=3), name="iTerm")
curve_pid_d_term = plot_02.plot(pen=pg.mkPen('m', width=3), name="dTerm")
curve_pid_output = plot_02.plot(pen=pg.mkPen('k', width=3), name="Output")

curve_errors_air_temp = plot_03.plot(pen=pg.mkPen('r', width=3), name="Air temp error")
curve_errors_heater_temp = plot_13.plot(pen=pg.mkPen('g', width=3), name="Heater temp error")

# curve_autopid_output = plot_03.plot(pen=pg.mkPen('g', width=3), name="Output")
# curve_autopid_heater_temp = plot_04.plot(pen=pg.mkPen('r', width=3), name="Heater temp")

def update():
    global time_data, s_data

    try:
        with open(log_file, "r") as f:
          lines = f.readlines()
          
        lines = lines[1:-1]
            
        columns = defaultdict(list)
        first_line = lines[0].strip().replace(":", "").split(" ")
        columnsCount = len(first_line)

        for lineIndex, line in enumerate(lines):
          line_s = line.strip().replace(":", "").split(" ")
          
          if (len(line_s) != columnsCount):
            continue
        
          row = defaultdict(float)
          
          try:
            for columnIndex, value in enumerate(line_s):
                if (columnIndex % 2 != 0):
                    continue
                
                columnName = value
                columnValue = float(line_s[columnIndex + 1])
                
                row[columnName] = columnValue
                
            for columnName, columnValue in row.items():
                columns[columnName].append(columnValue)
          except Exception as e:
            print("line parse error at line", lineIndex, ":", e)
            continue
              
        columns["t"] = np.array(columns["t"]) - columns["t"][0]
        columns["t"] = columns["t"] / 1000.0 / 60.0
        
        if (columns["s"]):
          columns["ne"] = np.array(columns["s"]) - np.array(columns["n"])
          
        if (columns["at"]):
          columns["ae"] = 60.0 - np.array(columns["at"])
        
        curve_air_temp.setData(columns["t"], columns["at"])
        curve_setpoint.setData(columns["t"], columns["s"])
        curve_heater_temp.setData(columns["t"], columns["n"])
        
        curve_pid_p_term.setData(columns["t"], columns["pp"])
        curve_pid_i_term.setData(columns["t"], columns["pi"])
        curve_pid_d_term.setData(columns["t"], columns["pd"])
        curve_pid_output.setData(columns["t"], columns["po"])
        
        curve_errors_air_temp.setData(columns["t"], columns["ae"])
        curve_errors_heater_temp.setData(columns["t"], columns["ne"])
        
        # curve_autopid_heater_temp.setData(columns["t"], columns["n"])
        # curve_autopid_output.setData(columns["t"], columns["o"])

    except Exception as e:
        print("read error:", e)

update()

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(1000)

app.exec()