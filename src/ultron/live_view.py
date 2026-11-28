"""Single-threaded, rate-capped MuJoCo overview for software-rendered desktops."""
import math
import time


class DrawSchedule:
    def __init__(self,fps=5):
        if not math.isfinite(fps) or not 0<fps<=30:
            raise ValueError('Viewer FPS must be finite and in (0,30]')
        self.interval=1/fps
        self.next_draw=-math.inf

    def ready(self,now):
        if now<self.next_draw: return False
        self.next_draw=now+self.interval
        return True


class LiveView:
    """No background rendering loop; all window GL calls restore camera context."""
    def __init__(self,model,data,fps=5,*,title='ULTRON | kinematic B2 patrol | oracle detection | Esc to stop'):
        import glfw
        import mujoco
        self.glfw=glfw;self.mj=mujoco;self.model=model;self.data=data
        self.schedule=DrawSchedule(fps)
        # The offscreen GLFW camera context leaves VISIBLE=false behind.
        glfw.default_window_hints()
        glfw.window_hint(glfw.VISIBLE,glfw.TRUE)
        glfw.window_hint(glfw.RESIZABLE,glfw.FALSE)
        self.window=glfw.create_window(800,450,title,None,None)
        glfw.default_window_hints()
        if not self.window:
            raise RuntimeError('Cannot open MuJoCo window on DISPLAY; use the desktop terminal or headless mode')
        previous=glfw.get_current_context()
        try:
            glfw.make_context_current(self.window)
            glfw.swap_interval(0)
            self.scene=mujoco.MjvScene(model,maxgeom=1000)
            self.context=mujoco.MjrContext(model,mujoco.mjtFontScale.mjFONTSCALE_100)
            self.camera=mujoco.MjvCamera()
            camera_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_CAMERA,'overview')
            if camera_id >= 0:
                self.camera.type=mujoco.mjtCamera.mjCAMERA_FIXED
                self.camera.fixedcamid=camera_id
            else:
                self.camera.lookat[:]=model.stat.center
                self.camera.distance=1.5*model.stat.extent
                self.camera.azimuth=115
                self.camera.elevation=-58
            self.option=mujoco.MjvOption()
        except BaseException:
            glfw.destroy_window(self.window)
            raise
        finally:
            glfw.make_context_current(previous)

    def sync(self):
        glfw=self.glfw;mujoco=self.mj
        glfw.poll_events()
        if glfw.window_should_close(self.window) or glfw.get_key(self.window,glfw.KEY_ESCAPE)==glfw.PRESS:
            raise RuntimeError('Viewer closed: patrol stopped; partial evidence retained')
        if not self.schedule.ready(time.monotonic()): return
        previous=glfw.get_current_context()
        try:
            glfw.make_context_current(self.window)
            width,height=glfw.get_framebuffer_size(self.window)
            if width and height:
                mujoco.mjv_updateScene(self.model,self.data,self.option,None,self.camera,mujoco.mjtCatBit.mjCAT_ALL,self.scene)
                for flag in (mujoco.mjtRndFlag.mjRND_SHADOW,mujoco.mjtRndFlag.mjRND_REFLECTION):
                    self.scene.flags[flag]=False
                # Bound physical framebuffer pixels too, including HiDPI scaling.
                mujoco.mjr_render(mujoco.MjrRect(0,0,min(width,800),min(height,450)),self.scene,self.context)
                glfw.swap_buffers(self.window)
        finally:
            glfw.make_context_current(previous)

    def close(self):
        previous=self.glfw.get_current_context()
        try:
            self.glfw.make_context_current(self.window)
            self.context.free()
        finally:
            self.glfw.make_context_current(previous)
            self.glfw.destroy_window(self.window)
