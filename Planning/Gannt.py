import datetime as dt
import gantt as gt

gt.define_font_attributes(fill='black', stroke='black', stroke_width=0, font_family="Verdana")

rHLR = gt.Resource('HLR')
rVV = gt.Resource('VV')
rMMM = gt.Resource('MMM')

tasks = []
def makeTask(sTag, iDay, iMonth, iNumDays, iPercent, aOwners):
    tasks.append(gt.Task(name=sTag,
                         start=dt.date(2022, iMonth, iDay),
                         duration=iNumDays,
                         percent_done=iPercent,
                         resources=aOwners,
                         color='violet'))

def makeSubtask(sTag, iDay, iMonth, iNumDays, iPercent, aOwners):
    tasks.append(gt.Task(name=sTag,
                         start=dt.date(2022, iMonth, iDay),
                         duration=iNumDays,
                         percent_done=iPercent,
                         resources=aOwners,
                         color='lightgreen'))

def makeMilestone(sTag, aDepends):
    tasks.append(gt.Milestone(name=sTag,
                              depends_of=aDepends))

def makeProject(sTag, aTasks):
    project = gt.Project(name=sTag)
    for i in range(len(aTasks)):
        project.add_task(aTasks[i])
    return project

#MAIN #7 (0-6)
makeTask('MAIN PROGRAM (C++)', 31, 10, 20, 50, [rHLR, rVV, rMMM])

makeSubtask('Structure', 31, 10, 8, 60, [rVV])
makeSubtask('Manual control (arduino)', 31, 10, 10, 90, [rVV, rMMM])
makeSubtask('Input/output handling', 2, 11, 10, 20, [rVV])
makeSubtask('Object recognition (openCV)', 7, 11, 13, 0, [rMMM])
makeSubtask('Movement', 9, 11, 11, 20, [rVV])
makeSubtask('Lifting and placing', 14, 11, 8, 0, [rVV])


#GUI #7 (7-13)
makeTask('GUI (C++, cvUI)', 31, 10, 20, 80, [rMMM])

makeSubtask('Video display', 31, 10, 5, 100, [rMMM])
makeSubtask('Buttons and meters', 31, 10, 8, 100, [rMMM])
makeSubtask('Input handling', 2, 11, 13, 50, [rMMM])        # Structure the inputs
makeSubtask('Error handling', 14, 11, 6, 10, [rMMM])         # Some display of error


#rPI #6 (14-19)
makeTask('RASPBERRY PI (python)', 31, 10, 20, 25, [rHLR])

makeSubtask('Contact and test values', 31, 10, 6, 100, [rVV])
makeSubtask('Input handling', 31, 10, 8, 30, [rHLR])
makeSubtask('Crash control', 2, 11, 8, 0, [rHLR])
makeSubtask('Video transmission', 2, 11, 8, 50, [rMMM])
makeSubtask('Sensor data transmission', 7, 11, 10, 10, [rHLR])


#Phys.Design #4 (20-23)
makeTask('PHYSICAL DESIGN', 31, 10, 20, 30, [rHLR, rVV, rMMM])

makeSubtask('Design lifting mecanism', 31, 10, 7, 23, [rVV])
makeSubtask('Design manual control', 7, 11, 5, 80, [rMMM])
makeSubtask('3D-print', 9, 11, 8, 20, [rVV, rMMM])
makeSubtask('Mounting', 14, 11, 6, 0, [rHLR, rVV, rMMM])



makeTask('MEETINGS AND REPORTS', 31, 10, 30, 0, [rHLR, rVV, rMMM])

makeSubtask('Prog.Rep.', 1, 11, 2, 100, [rHLR, rVV, rMMM])
makeSubtask('Meeting', 9, 11, 1, 100, [rHLR, rVV, rMMM])
makeSubtask('Prog.Rep.', 15, 11, 2, 100, [rHLR, rVV, rMMM])
makeSubtask('Meeting', 23, 11, 1, 0, [rHLR, rVV, rMMM])
makeSubtask('Project report', 24, 11, 10, 0, [rHLR, rVV, rMMM])
makeSubtask('Prog.Rep.', 29, 11, 2, 0, [rHLR, rVV, rMMM])
makeSubtask('Meeting', 7, 12, 1, 0, [rHLR, rVV, rMMM])

makeMilestone('Working GUI', [tasks[10], tasks[11]])
makeMilestone('Autonomic operations', [tasks[4], tasks[5], tasks[14]])
makeMilestone('Finished project report', [tasks[0], tasks[7], tasks[12], tasks[18], tasks[32]])




pRVR = makeProject('Sphero RVR', tasks)
pRVR.make_svg_for_tasks(filename='w46.svg', today=dt.date.today())

if (__name__ == '__main__'):
    print(dt.date.today())




