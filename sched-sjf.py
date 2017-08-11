#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-



######################################################################################################
#                                                                                                    #
# Codestellen, die die Gruppe Korbinian Karl, Ehsan Moslehi und Sebastian Baumann angefertigt haben, #
# sind mit der Zeichenfolge "# ####Gruppe:" gekennzeichnet                                           #
# Zum testen von SJF-Scheduling haben wir unsere Testdatei nur-cpu2.dat und nur-cpu.dat (im Ordner)  #
# verwendet.                                                                                         #
#                                                                                                    #
######################################################################################################



# sched.py (Praktikum, Arbeitsblatt 7) !!!!!!!!!!!!!!!!! SJF !!!!!!!!!!!!!!!!!
# v2.2 (2011/05/12)
# 
# Vorlesung Betriebssysteme
# Hans-Georg Eßer, Hochschule München
# hans-georg.esser@hm.edu

proccount = 0;   # Anzahl der Prozesse im System

# Status-Konstanten
S_ACTIVE=1
S_READY=2
S_BLOCKED=3
S_DONE=0

def statusname (i):
  if   i == 1: return "Active"
  elif i == 2: return "Ready"
  elif i == 3: return "Blockd"
  elif i == 0: return "Done"
  else: return "Error"

tasks = []     # Liste der Tasks
current = -1   # aktueller Prozess
trace = []     # Wann lief welcher Prozess?

runqueue = []  # Prozess-IDs, die bereit sind (Warteschlange)
blocked  = []  # Prozess-IDs, die blockiert sind

# Methoden zum Setzen und Abfragen von Prozesseigenschaften

def set_cputime (pid, cputime):    tasks[pid]["cputime"] = cputime
def set_iotime (pid, iotime):      tasks[pid]["iotime"] = iotime
def set_status (pid, status):      tasks[pid]["status"] = status
def set_endtime (pid, endtime):    tasks[pid]["endtime"] = endtime
def set_firstruntime (pid,t):      tasks[pid]["firstruntime"] = t
def get_starttime (pid):    return tasks[pid]["starttime"]
def get_endtime (pid):      return tasks[pid]["endtime"]
def get_cputime (pid):      return tasks[pid]["cputime"]
def get_iotime (pid):       return tasks[pid]["iotime"]
def get_status (pid):       return tasks[pid]["status"]
def get_firstruntime(pid):  return tasks[pid]["firstruntime"]
def inc_cputime (pid):             set_cputime (pid, get_cputime(pid)+1)
def inc_iotime (pid):              set_iotime (pid, get_iotime(pid)+1)
def get_head_behavior(pid): return tasks[pid]["behavior"][0]
def dec_head_behavior(pid):        tasks[pid]["behavior"][0] -= 1
def crop_head_behavior(pid):       tasks[pid]["behavior"] = tasks[pid]["behavior"][1:]


def sortList(l, k):
  # Sortiert eine als Parameter übergebene Liste l nach einem als Parameter übergebenem Schlüsselwort k
  # und liefert anschließend die sortierte Liste wieder zurück
  return sorted(l, key=lambda n: n[k])


def activate (p):
  # Prozess mit der angegebenen PID aktivieren (also status auf
  # S_ACTIVE setzen)
  global proccount
  if p >= 0:
    for pid in range(0,proccount):
      if get_status (pid) == S_ACTIVE: 
        set_status(pid, S_READY)
    set_status(p, S_ACTIVE)

def get_freepid():
  # naechste freie PID berechnen und zurueckgeben; Prozess-Zaehler
  # proccount erhoehen
  global proccount
  proccount+=1
  return proccount-1

def create_process (starttime, behavior):
  # Prozess in Prozessliste eintragen (auch wenn er einen in
  # der Zukunft liegenden Startzeitpunkt hat)
  global tasks
  pid = get_freepid()
  task={}  # neuer Task ohne Eigenschaften (leeres Dictionary)
  task["starttime"] = starttime
  task["firstruntime"] = -1   # noch nie gelaufen
  status=S_READY  # Standard
  if behavior[0]==0:
    # Prozess faengt mit I/O-Phase an
    behavior = behavior[1:]
    status=S_BLOCKED
  task["behavior"] = behavior
  tasks.append(task)
  # den Rest machen wir ueber die definierten Funktionen
  set_cputime(pid,0)
  set_iotime(pid,0)
  set_status(pid,status)
  set_endtime(pid,-1)  # noch nicht beendet
  return pid

def futureprocesses(t):
  # gibt alle PIDs zurueck fuer Prozesse, die nach Zeit t starten
  global proccount
  fp = []
  for pid in range(0,proccount):
      if get_starttime(pid) > t: fp.append(pid)
  return fp


# schedule() nach SJF
def schedule():
  # Implementation des SJF-Schedulers (Shortest Job First)
  global current, tasks, runqueue, blocked, current, cputime

  # falls aktueller Prozess noch bereit: weitermachen (SJF)
  if (current >= 0) and (get_status(current) == S_ACTIVE):
    choice = current
    # falls weder bereite noch blockierte Prozesse: Ende  
  elif runqueue + blocked + futureprocesses(cputime) == []:
    choice = -2
  elif (runqueue != []):   # ####Gruppe: prüft, ob die Run Queue leer ist, oder nicht
    choice = runqueue[0]   # ####Gruppe: choice wird zuerst auf den ersten bereiten Prozess in der runqueue gesetzt
    for i in runqueue[1:]:   # ####Gruppe: In einer Schleife werden die Elemente der runqueue durchiteriert (beginnend, beim zweiten Element in der runqueue, da choice ja schon auf den ersten gesetzt wurde)
      if (get_head_behavior(i) < get_head_behavior(choice)):   # ####Gruppe: Falls die CPU-Time des gefundenen Elementes in der runqueue kleiner ist, als die CPU-Time des Choice-Prozesses,
        choice = i   # ####Gruppe: So ist der neue Prozess choice der gefundene Prozess in der Run Queue
  else:
    # falls alle blockiert: idlen!
    choice = -1  #  alle blockiert
  return choice




def ps ():
  global tasks,proccount,cputime,runqueue,blocked
  print "PID | Sta | End | CPU | I/O | Status | Verhalten"
  for pid in range(0,proccount):
    if 1<2: # cputime >= get_starttime(pid)-1:
      task = tasks[pid]
      print "%3d | %3d | %3d | %3d | %3d | %6s |" % (pid, 
        get_starttime(pid), get_endtime(pid),
        get_cputime(pid), get_iotime(pid), 
        statusname( get_status(pid) ) ),
      print task["behavior"]                         
  print "Runqueue:",runqueue,"  Blocked:",blocked
  print 
  return

def init ():
  from sys import argv  # Argumente auswerten; Name der Konf.-Datei
  from sys import exit
  try:
    filename = argv[1]
    f = open (filename, "r")
    lines = f.readlines()
    f.close ()
  except:
    print "Fehler: Bitte Dateinamen der Konfigurationsdatei angeben."
    exit()
  
  for l in lines:
    # Eine Zeile der Form
    #   "0:5,10,5,10,-1"
    # zerlegen in:
    #   starttime = 0
    #   behavior = [5,10,5,10,-1]
    # und create_process() mit diesen beiden Argumenten aufrufen
    if ":" not in l: return   ###if l == "\n": return
    (starttime,times) = l[:-1].split(":")
    starttime = int(starttime)
    times = times.split(",")
    behavior=[]
    for t in times:
      behavior.append(int(t))
    create_process (starttime, behavior)
  return


def update_blocked_processes():
  global current, cputime, runqueue, blocked
  for pid in blocked:
    # Prozess nur bearbeiten, wenn er bereits im System ist (1. Bedingung)
    # und wenn er nicht gerade erst in die blocked queue geschoben wurde
    # (2. Bedingung)
    ##if (cputime >= get_starttime(pid)) and (pid != current):
    if pid != current:
      dec_head_behavior(pid)
      inc_iotime(pid)
      if get_head_behavior(pid) == 0:
        # Aktuelle I/O-Phase abgeschlossen
        crop_head_behavior(pid)
        set_status(pid,S_READY)
        blocked.remove(pid)
        if get_head_behavior(pid) == -1:
          set_status(pid,S_DONE)
          set_endtime(pid,cputime)
        else:
          runqueue.append(pid)
  return


def run_current():
  # Aktuellen Prozess eine Zeiteinheit rechnen lassen
  global current, cputime, runqueue, blocked
  if get_firstruntime(current) == -1:   # falls Prozess zum ersten Mal laeuft:
    set_firstruntime(current, cputime)
  dec_head_behavior(current)
  inc_cputime(current)
  if get_head_behavior(current) == 0:
    crop_head_behavior(current)
    set_status(current,S_BLOCKED)
    runqueue.remove(current)
    if get_head_behavior(current) == -1:
      set_status(current,S_DONE)
      set_endtime(current,cputime)
      if current in runqueue: runqueue.remove(current)
    else:
      blocked.append(current)
  return


def log_to_trace(status):
  global trace
  if status == -1: status = "NONE"
  elif status == -2: status = "END"
  trace.append(status)


def stats():
  global cputime
  print 
  print "Endzeit: %d" % cputime
  print "Trace:",trace

  print "Laufzeiten:"
  print "PID Ankunft Rechenz Startze Endzeit TurnAro  Quotient"
  print "-----------------------------------------------------"

  for pid in range(0,proccount):
    stime  = get_starttime(pid)
    etime  = get_endtime(pid)+1
    frtime = get_firstruntime(pid)
    cputime = get_cputime(pid)
    if cputime != 0:
      ratio = (etime-stime+0.0)/cputime
    else:
      ratio = -1
    print "%3d %7d %7d %7d %7d %7d %9.4f" % (pid, 
      stime, cputime, frtime, etime, etime-stime, ratio)




##### Hauptprogramm #####


init ()
print "Anzahl tasks: ", proccount


cputime = 0   # Init. der Uhr


ps()  # Prozessliste vor dem Start

# Hauptschleife
finished = 0
while not finished:

  # neue Prozesse angekommen?
  for pid in range(0,proccount):
    if get_starttime(pid) == cputime:
      # jetzt an die richtige Liste anhaengen
      if get_status(pid)==S_BLOCKED:
        blocked.append(pid)
        # Prozess, der mit I/O anfaengt: firstruntime = starttime
        set_firstruntime (pid,cputime)
      else:
        runqueue.append(pid)

  current = schedule()    # Scheduler aufrufen
  log_to_trace (current)  # Auswahl protokollieren
  activate (current)      # Status aendern
  print "current", current
  if current >= 0:
    print "Zeit: %4d, Es laeuft PID: %3d [%3d]" % (cputime,current,get_cputime(current))
    run_current()  # aktuellen Prozess ausfuehren
    # bei allen blockierten Prozesse Wartezeit runterzaehlen
    update_blocked_processes()
  elif current == -1:
    print "Zeit: %4d, alle blockiert" % cputime
    # bei allen blockierten Prozesse Wartezeit runterzaehlen
    update_blocked_processes()
  else:  
    finished = 1
    break
  ps()               # Prozessinformationen ausgeben
  cputime += 1       # CPU-Zeit erhoehen

stats()
