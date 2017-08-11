#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

# sched.py (Praktikum, Arbeitsblatt 7)
# v2.2 (2011/05/12)
# 
# Vorlesung Betriebssysteme
# Hans-Georg Eßer, Hochschule München
# hans-georg.esser@hm.edu

proccount = 0;   # Anzahl der Prozesse im System

# Status-Konstanten
S_ACTIVE=1    # SB: Prozess-Status "aktiv", bedeutet, der Prozess ist vorbereitet und kann nun in die Prozess-Abarbeitung eintreten
S_READY=2     # SB: Prozess-Status "ready", bedeutet, der Prozess kann jederzeit ausgeführt werden
S_BLOCKED=3   # SB: Prozess-Status "blockiert", bedeutet der Prozess ist blockiert und wartet auf I/O-Zeit
S_DONE=0      # SB: Prozess-Status "fertig", bedeutet der Prozess ist komplett abgearbeitet

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

# SB: Zum besseren Verständnis: ein Prozess ist folgendermaßen aufgebaut:
# SB: 0: 100, 6, -1
# SB: 0 -> ist der Startzeitpunkt eines Prozesses
# SB: 100 -> Ist die benötigte Prozessor-Zeit eines Prozesses
# SB: 6 -> ist die benötigte I/O-Zeit eines Prozesses
# SB: -1 -> das Endezeichen eines Prozesses

# SB: Beispielsweise verkürzt der Befehl crop_head_behavior(Prozess 0) den oben aufgeführten Prozess um den Eintrag an Position 0:
# SB: Aus 0: 100, 6, -1 wird 0: 6, -1 und die benötigte Prozessor-Zeit von 100 Einheiten ist abgearbeitet
# SB: Wird die Liste auf bis zu 0: -1 gekürzt, so ist der ganze Prozess abgearbeitet

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
def dec_head_behavior(pid):        tasks[pid]["behavior"][0] -= 1                         # SB: verringert den Wert an Position [pid]["behavior"][0] um 1: aus 0: 100, 6, -1 wird 0: 99, 6, -1
def crop_head_behavior(pid):       tasks[pid]["behavior"] = tasks[pid]["behavior"][1:]    # SB: Verkürzt die Liste der abzuarbeitenden Aktionen eines Prozesses um EINE Aktion (die erste, also Position 0)

def activate (p):
  # Prozess mit der angegebenen PID aktivieren (also status auf
  # S_ACTIVE setzen)
  global proccount
  if p >= 0:   # SB: Prüft, ob der übergene Prozess überhaqupt ein gültiger Prozess (PID >= 0) ist
    for pid in range(0,proccount):   # SB: for-Schleife läuft so oft, wie es im Programm angemeldete Prozesse gibt
      if get_status (pid) == S_ACTIVE:   # SB: Falls der Status des gerade gefundenen Prozesses schon ACTIVE ist
        set_status(pid, S_READY)   # SB: wird der Status des Prozesses auf READY gesetzt und er kann ausgeführt werden
    set_status(p, S_ACTIVE)   # SB: Falls Prozess nicht ACTIVE ist, so wird er auf ACTIVE gesetzt und kann bearbeitet werden

def get_freepid():
  # naechste freie PID berechnen und zurueckgeben; Prozess-Zaehler
  # proccount erhoehen
  global proccount
  proccount+=1
  return proccount-1

def create_process (starttime, behavior):                                                 # SB: als Parameter benötigt diese Methode die Startzeit und die Aktionen des zu erzeugenden Prozesses
  # Prozess in Prozessliste eintragen (auch wenn er einen in
  # der Zukunft liegenden Startzeitpunkt hat)
  global tasks                                                                            # SB: Zugriff auf global deklarierte Variablen
  pid = get_freepid()                                                                     # SB: Erzeugt eine neue PID für den zu erzeugenden neuen Prozess
  task={}  # neuer Task ohne Eigenschaften (leeres Dictionary)
  task["starttime"] = starttime                                                           # SB: in die Liste task wird nun das Bezeichner-Wert-Paar "starttime: 0" eingetragen
  task["firstruntime"] = -1   # noch nie gelaufen                                         # SB: in die Liste task wird nun das Bezeichner-Wert-Paar "firstruntime: -1" am Anfang eingetragen
  status=S_READY  # Standard                                                              # SB: Der Status des aktuellen Prozesses wird auf BEREIT (READY) gesetzt
  if behavior[0]==0:                                                                      # SB: Sollte übrige Zeit der Prozessor-Aktion an erster Position 0 sein, fängt Prozess mit I/O-Phase an
    # Prozess faengt mit I/O-Phase an
    behavior = behavior[1:]                                                               # SB: Die 0 an erster Position der Prozessor-Aktionen wird entfernt
    status=S_BLOCKED                                                                      # SB: Der Prozessor hat keine CPU-Zeit, sondern eine I/O-Phase und dessen Status muss daher auf BLOCKED gesetzt werden
  task["behavior"] = behavior                                                             # SB: in die Liste task wird nun das Bezeichner-Wert-Paar "behavior: übergeben behavior" eingetragen
  tasks.append(task)                                                                      # SB: die Liste tasks, in der alle erzeugten Prozesse gespeichert werden, wird die task-Liste des neuen Prozesses angefügt
  # den Rest machen wir ueber die definierten Funktionen
  set_cputime(pid,0)                                                                      # SB: in die Liste task wird nun das Bezeichner-Wert-Paar "cputime: 0" eingetragen 
  set_iotime(pid,0)                                                                       # SB: in die Liste task wird nun das Bezeichner-Wert-Paar "iotime: 0" eingetragen
  set_status(pid,status)                                                                  # SB: in die Liste task wird nun das Bezeichner-Wert-Paar "status: aktueller status" eingetragen
  set_endtime(pid,-1)  # noch nicht beendet                                               # SB: in die Liste task wird nun das Bezeichner-Wert-Paar "endtime: -1" eingetragen (noch nicht beendet)
  return pid                                                                              # SB: die pid des erzeugten Prozesses wird zurückgegeben

def futureprocesses(t):
  # gibt alle PIDs zurueck fuer Prozesse, die nach Zeit t starten
  global proccount
  fp = []
  for pid in range(0,proccount):
      if get_starttime(pid) > t: fp.append(pid)
  return fp

def schedule():
  # Implementation des FCFS-Schedulers (First Come, First Served)
  global current, tasks, runqueue, blocked, current, cputime
  
  # falls aktueller Prozess noch bereit: weitermachen (FCFS)
  if (current >= 0) and (get_status(current) == S_ACTIVE):   # SB: Falls der aktuelle Prozess im System ist (>= 0) UND er den Status ACTIVE hat
    choice = current   # SB: wird der aktuelle Prozess ausgewählt und zurückgegeben
  # falls weder bereite noch blockierte Prozesse: Ende
  elif runqueue + blocked + futureprocesses(cputime) == []:   # SB: Falls alle Queues (Run, Blocked und Futureprocess) leer sind, sind alle im System existenten Prozese abgearbeitet
    choice = -2   # SB: und es wird -2 als Fluchtwert zurückgegeben (log_to_trace loggt den Prozess als "END") und finish (unten im Hauptprogramm) wird auf 1 gesetzt und damit beendet
  # falls nicht: nehme ersten Prozess aus Runqueue
  elif (runqueue != []):   # SB: Die Run Queue ist nicht leer und es wird ganz nach FCFS der erste sich in der Run Queue befindende Prozess ausgewählt und zurückgegeben
    choice = runqueue[0]   # Erster Prozess in Run Queue wird zurückgegeben
  else:
  # falls alle blockiert sind: idlen!
    choice = -1  #  alle blockiert # SB: liefert -1 zurück und meldet, dass alle Prozesse gerade auf I/O-Eingaben warten
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
  global current, cputime, runqueue, blocked   # SB: Zugriff auf global deklarierte Variablen
  for pid in blocked:   # SB: Schleife, die durch alle Prozesse in der Blocked Queue iteriert
    # Prozess nur bearbeiten, wenn er bereits im System ist (1. Bedingung)
    # und wenn er nicht gerade erst in die blocked queue geschoben wurde
    # (2. Bedingung)
    ##if (cputime >= get_starttime(pid)) and (pid != current):
    if pid != current:   # SB: falls der gefundene Prozess in der Blocked Queue nicht dem aktuellen Prozess entspricht, dann
      dec_head_behavior(pid)   # verbleibende I/O-Wartezeit um 1 Einheit reduzieren
      inc_iotime(pid)   # SB: Erhöht abgelaufene I/O-Wartezeit um 1 für den Log
      if get_head_behavior(pid) == 0:   # SB: falls die I/O-Wartezeit des des gefundenen Prozesses abgearbeitet ist:
        crop_head_behavior(pid)   # SB: Entfernt die Prozessor-Aktion I/O-Wartezeit aus der Aktionen-Liste des Prozesses
        set_status(pid,S_READY)   # SB: setzt den Prozess-Status des gefundenen Prozesses wieder auf READY (kann wieder Prozessor-Zeit haben)
        blocked.remove(pid)   # SB: entfernt den gefundenen Prozess dann aus der Blocked Queue
        if get_head_behavior(pid) == -1:   # SB: falls die Prozessor-Aktion nach crop -1 ist, so ist Prozess abgearbeitet
          set_status(pid,S_DONE)   # SB: der aktuelle Prozess soll dann in den Status DONE gesetzt werden
          set_endtime(pid,cputime)   # SB: der Prozess ist beendet und dessen Endzeit-Wert muss nun auf die aktuelle CPU-Zeit gesetzt werden
        else:   # SB: Die Prozessor-Aktionen sind noch nicht alle abgearbeitet, der Prozess hatte eben eine I/O-Wartezeit, er wird daher in die Run Queue hinzugefügt
          runqueue.append(pid)   # SB: fügt Prozess am Ende der Run Queue hinzu, um die verbleibende Prozessor-Zeit abzuarbeiten
  return


def run_current():
  # Aktuellen Prozess eine Zeiteinheit rechnen lassen
  global current, cputime, runqueue, blocked   # SB: Zugriff auf global deklarierte Variablen
  if get_firstruntime(current) == -1:   # falls Prozess zum ersten Mal laeuft (firstruntime == -1)
    set_firstruntime(current, cputime)   # SB: soll die firstruntime auf die alktuelle CPU-Zeit gesetzt werden
  dec_head_behavior(current)   # verbleibende Rechenzeit (CPU-Time) um 1 reduzieren
  inc_cputime(current)   # SB: erhöht die abgelaufene CPU-Zeit um 1 (ist die Zeit, die der Prozess bereits Prozessor-Zeit hatte)
  if get_head_behavior(current) == 0:   # SB: falls der Prozess bereits abgearbeitet ist (behavior == 0)
    crop_head_behavior(current)   # SB: so entferne die abgearbeitete Prozessor-Aktion aus der Aktionen-Liste des aktuellen Prozesses
    set_status(current,S_BLOCKED)   # SB: jeder Prozess hat zuerst Rechenzeit auf Prozessor und dann wartet er auf I/O-Eingaben. Prozessor-Zeit ist abgearbeitet, Prozess wird blocked gesetzt, um I/O-Zeit abzuwarten
    runqueue.remove(current)   # SB: der aktuelle Prozess wird danach aus der Run Queue entfernt
    if get_head_behavior(current) == -1:  # SB: falls die Prozessor-Aktion nach crop -1 ist, so ist Prozess abgearbeitet
      set_status(current,S_DONE)   # SB: der aktuelle Prozess soll dann in den Status DONE gesetzt werden
      set_endtime(current,cputime)   # SB: der Prozess ist beendet und dessen Endzeit-Wert muss nun auf die aktuelle CPU-Zeit gesetzt werden
      if current in runqueue:   # SB: Sollte der fertig abgearbeitete Prozess noch in der Run Queue sein, muss er entfernt werden
        runqueue.remove(current)   # SB: Prozess wird aus Run Queue entfernt
    else:   # SB: Die Prozessor-Aktionen sind noch nicht abgearbeitet, der Prozess hatte eben eine Prozessor-Zeit, er wird daher auf blocked gesetzt, um auf I/O-Eingaben zu warten
      blocked.append(current)   # SB: fügt Prozess am Ende der Blocked Queue hinzu, um die verbleibende I/O-Wartezeit abzuarbeiten
  return


def log_to_trace(status):
  global trace
  if status == -1: status = "NONE"   # SB: Falls als Status (also als aktueller Prozess) -1 geliefert wird, wird NONE statt der Prozess-Nummer geloggt
  elif status == -2: status = "END"   # SB: Falls als Status (also als aktueller Prozess) -2 geliefert wird, so wird END statt der Prozess-Nummer geloggt
  trace.append(status)   # SB: der aktuelle Status (also die Prozess-Nummer, NONE oder END) wird dem Log hinzugefügt


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
finished = 0   # SB: Hilfsvariable, die speichert, ob alle Prozesse abgearbeitet sind
# SB: In while-Schleife werden die Prozesse nacheinander in der Reihenfolge, die der schedule() liefert, abgearbeitet
while not finished:   # SB: while endet, falls finished irgendwas anderes als 0 ist!

  # neue Prozesse angekommen?
  for pid in range(0,proccount):   # SB: zählt die pids von 0 bis zu proccount durch. Das geht so lange, wie es pids mit einem Wert > 0 gibt. Ist ein Prozess abgearbeitet, so wird er negativ gesetzt!
    if get_starttime(pid) == cputime:   # SB: falls get_starttime des pids gleich der aktuellen cputime ist, ist der Prozess an der Reihe und kann starten
      if get_status(pid)==S_BLOCKED:   # SB: Status des Prozesses wird ausgelesen. Falls der Status BLOCKED ist, dann wird er in die Blocked Queue am Ende angehängt
        blocked.append(pid)   # SB: hängt den aktuellen Prozess in die Liste blocked ein (am Ende)
        # Prozess, der mit I/O anfaengt: firstruntime = starttime
        set_firstruntime (pid,cputime)   # SB: legt den Zeitpunkt fest, zu welcher CPU-Zeit die Ausführung des aktuellen Prozesses begonnen hat -> Startzeit für Log am Ende der Ausführung
      else:
        runqueue.append(pid)   # SB: Sollte der Status des Prozesses NICHT BLOCKED sein, so wird er am Ende der Run Queue hinzugefügt und ist bereit zur Ausführung

  current = schedule()    # Scheduler aufrufen und nächsten Prozess, der ausgeführt werden soll, ermitteln
  log_to_trace (current)  # Auswahl protokollieren # SB: der Prozess, den der Scheduler ausgewählt hat, wir geloggt und am Ende der Ausführung angezeigt
  activate (current)      # Status aendern   # SB: ACTIVE-Prozess wird zu READY und jeder andere wird zu ACTIVE
  if current >= 0:   # SB: Nur falls der Prozess auch wirklich ein Prozess ist (PID >= 0), so wird hier bearbeitet
    print "Zeit: %4d, Es laeuft PID: %3d [%3d]" % (cputime,current,get_cputime(current))   # SB: Ausgabe aktuelle CPU-Zeit, aktueller Prozess und Zeit, die der Prozess schon auf dem Prozessor hatte
    run_current()  # aktuellen Prozess ausfuehren
    update_blocked_processes()   # bei allen blockierten Prozesse Wartezeit runterzaehlen
  elif current == -1:   # SB: es gibt im Augenblick keinen Prozess, der abgearbeitet werden kann -> alle Prozesse sind gerade geblockt
    print "Zeit: %4d, alle blockiert" % cputime
    # bei allen blockierten Prozessen Wartezeit runterzaehlen
    update_blocked_processes()
  else:  
    finished = 1   # SB: setztn finish auf 1 und damit wird die while-Schleife verlassen. Alle im System existenten Prozesse sind abgearbeitet, das Programm kann beendet werden
    break
  ps()               # Prozessinformationen ausgeben
  cputime += 1       # CPU-Zeit erhoehen

stats()
