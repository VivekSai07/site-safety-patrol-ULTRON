"""In-process publish/subscribe used to push live updates to browsers (Server-Sent Events)."""
import asyncio
import json


class Bus:
    def __init__(self, queue_size=200):
        self.queue_size = queue_size
        self.subscribers = set()
        self.loop = None  # set at server start; lets worker threads publish safely

    def attach(self, loop):
        self.loop = loop

    def subscribe(self):
        queue = asyncio.Queue(self.queue_size)
        self.subscribers.add(queue)
        return queue

    def unsubscribe(self, queue):
        self.subscribers.discard(queue)

    def publish(self, kind, data):
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if self.loop is not None and running is not self.loop:
            self.loop.call_soon_threadsafe(self._deliver, kind, data)
        else:
            self._deliver(kind, data)

    def _deliver(self, kind, data):
        message = {'kind': kind, 'data': data}
        for queue in list(self.subscribers):
            try:
                queue.put_nowait(message)
            except asyncio.QueueFull:
                # A browser that stops reading is dropped; it reconnects and reloads state.
                self.subscribers.discard(queue)


def sse_format(message):
    return f"event: {message['kind']}\ndata: {json.dumps(message['data'])}\n\n"


async def sse_stream(bus, is_disconnected, keepalive_s=15.0):
    queue = bus.subscribe()
    try:
        yield ': connected\n\n'
        while not await is_disconnected():
            try:
                message = await asyncio.wait_for(queue.get(), keepalive_s)
            except asyncio.TimeoutError:
                yield ': keepalive\n\n'
                continue
            yield sse_format(message)
    finally:
        bus.unsubscribe(queue)
