from psutil import process_iter,AccessDenied
from signal import SIGTERM # or SIGKILL

def kill(port):

    for proc in process_iter():
        for conns in proc.connections(kind='inet'):
            if conns.laddr.port == port:
                try:
                    proc.send_signal(SIGTERM) # or SIGKILL
            
                # except AccessDenied:
                except (PermissionError, AccessDenied) as e:
                    print( "Process is not allowing us to view the CPU Usage!" )
                    print(e)

                
    return 1
if __name__ == "__main__":
    kill(8081)

# for proc in psutil.process_iter():
#    try:
#        if proc.name == PROCNAME:
#           print(proc)
#    except (PermissionError, AccessDenied):
#        print "Permission error or access denied on process" # can't display name or id here