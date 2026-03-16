import os

class PIDLogger:

    def __init__(self, filename="pid_tests.csv"):
        self.filename = filename

        # create file and header if doesn't exist
        if filename not in os.listdir():
            with open(filename, "w") as f:
                f.write("run,Kp,Ki,Kd,time_ms,setpoint,position,error,output\n")

    def log(self, run, Kp, Ki, Kd, time_ms, setpoint, position, error, output):

        line = "{},{},{},{},{},{},{},{},{}\n".format(
            run, Kp, Ki, Kd,
            time_ms, setpoint, position, error, output
        )

        with open(self.filename, "a") as f:
            f.write(line)

