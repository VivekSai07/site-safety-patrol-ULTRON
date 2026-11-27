import unittest
from types import SimpleNamespace
from ultron.live_view import DrawSchedule,LiveView


class DrawScheduleTests(unittest.TestCase):
    def test_viewer_draws_are_capped_and_never_catch_up_in_a_burst(self):
        schedule=DrawSchedule(5)
        self.assertTrue(schedule.ready(0))
        self.assertFalse(schedule.ready(.05))
        self.assertFalse(schedule.ready(.19))
        self.assertTrue(schedule.ready(.2))
        self.assertTrue(schedule.ready(10))
        self.assertFalse(schedule.ready(10))

    def test_invalid_or_excessive_viewer_rates_are_rejected(self):
        for fps in [0,-1,float('nan'),float('inf'),31]:
            with self.assertRaises(ValueError): DrawSchedule(fps)

    def make_view(self,render):
        current=['camera-context']
        view=LiveView.__new__(LiveView)
        view.glfw=SimpleNamespace(poll_events=lambda:None,window_should_close=lambda w:False,
            get_key=lambda *a:0,KEY_ESCAPE=256,PRESS=1,get_current_context=lambda:current[0],
            make_context_current=lambda c:current.__setitem__(0,c),
            get_framebuffer_size=lambda w:(3840,2160),swap_buffers=lambda w:None)
        view.mj=SimpleNamespace(mjv_updateScene=lambda *a:None,mjr_render=render,
            MjrRect=lambda *a:a,mjtCatBit=SimpleNamespace(mjCAT_ALL=1),
            mjtRndFlag=SimpleNamespace(mjRND_SHADOW=0,mjRND_REFLECTION=1))
        view.model=view.data=view.option=view.camera=view.context=None
        view.scene=SimpleNamespace(flags=[True,True]);view.window='window'
        view.schedule=DrawSchedule(5)
        return view,current

    def test_hidpi_draw_is_bounded_and_camera_context_restored(self):
        rects=[]
        view,current=self.make_view(lambda rect,*args:rects.append(rect))
        view.sync()
        self.assertEqual(rects,[(0,0,800,450)])
        self.assertEqual(current[0],'camera-context')

    def test_render_error_still_restores_camera_context(self):
        def fail(*args): raise RuntimeError('render failed')
        view,current=self.make_view(fail)
        with self.assertRaises(RuntimeError): view.sync()
        self.assertEqual(current[0],'camera-context')
