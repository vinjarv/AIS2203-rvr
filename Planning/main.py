
import datetime as dt
import gantt as gt

gt.define_font_attributes(fill='black', stroke='black', stroke_width=0, font_family="Verdana")



#Resources
rVV = gt.Resource('VV')
rHLR = gt.Resource('HLR')
rMMM = gt.Resource('MMM')

#Tasks
t1 = gt.Task(name='Lift mechanism', start=dt.date(2022, 10, 6), duration=8, percent_done=50, resources=[rVV], color='cyan')
t2 = gt.Task(name='uTask1', start=dt.date(2022, 10, 6), duration=3, percent_done=50, resources=[rVV], color='lightblue', depends_of=[t1])
t3 = gt.Task(name='uTask1', start=dt.date(2022, 10, 6), duration=3, percent_done=50, resources=[rVV], color='lightblue', depends_of=[t1])
t4 = gt.Task(name='uTask1', start=dt.date(2022, 10, 8), duration=3, percent_done=50, resources=[rVV], color='lightblue', depends_of=[t1])
t5 = gt.Task(name='uTask1', start=dt.date(2022, 10, 10), duration=5, percent_done=50, resources=[rVV], color='lightblue', depends_of=[t1])

t10 = gt.Task(name='Live streaming', start=dt.date(2022, 10, 7), duration=10, percent_done=10, resources=[rMMM], color='cyan')
t11 = gt.Task(name='Object recog', start=dt.date(2022, 11, 1), duration=10, percent_done=30, resources=[rHLR], color='cyan')
ms1 = gt.Milestone(name='name', depends_of=[t1])

#Project
pRVR = gt.Project(name='RVR')
pRVR.add_task(t1)
pRVR.add_task(t2)
pRVR.add_task(t10)
pRVR.add_task(t11)
pRVR.add_task(ms1)

pRVR.make_svg_for_tasks(filename='ganttTest1.svg', start=dt.date(2022,10,4))


if __name__ == '__main__':
    print('ok')


