import asyncio
import fastapi
import fastapi.responses
from collections import defaultdict
from uvicorn import Config, Server
from airdc.common.visualizers.basis import (
    SampleInfo,
    VisualizerBasis,
    WebVisualizerConfig,
)
from airdc.common.utils.progress import run_event_loop
from airdc.basis import DictDataStamped


class FastAPIVisualizer(VisualizerBasis):
    config: WebVisualizerConfig

    def on_configure(self) -> bool:
        self.app = fastapi.FastAPI()
        self.frames: dict[str, bytes] = {}
        self.info: SampleInfo = None
        self.events: dict[str, asyncio.Event] = defaultdict(asyncio.Event)
        self._setup_routes()
        self.prefix = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
        self.suffix = b"\r\n"
        self.server_future = asyncio.run_coroutine_threadsafe(
            self._server_task(), run_event_loop()
        )
        return True

    def _setup_routes(self):
        @self.app.get("/")
        def index():
            return fastapi.responses.HTMLResponse(self._html_index())

        @self.app.get("/stream/{stream_id}")
        async def stream(stream_id: str):
            return fastapi.responses.StreamingResponse(
                self._gen_frames(stream_id),
                media_type="multipart/x-mixed-replace; boundary=frame",
            )

        # 添加 favicon 路由：返回一个空的响应或默认图标
        @self.app.get("/favicon.ico")
        def favicon():
            return fastapi.responses.Response(status_code=204)

        @self.app.get("/info")
        def get_info():
            if self.info is not None:
                return {"index": self.info.index, "episode": self.info.episode}
            return {"index": -1, "episode": -1}

    def _html_index(self) -> str:
        freq = 20
        html = """\
        <!doctype html>
        <html lang='en'>
        <head>
            <meta charset='utf-8'>
            <title>Multi Stream</title>
            <style>
                .stream-container {
                    display: flex;
                    flex-wrap: wrap;
                    gap: 16px;
                    align-items: flex-start;
                }
                .stream-item {
                    flex: 0 0 auto;
                    max-width: 480px;
                    margin-bottom: 16px;
                }
                .stream-item img {
                    width: 100%;
                    height: auto;
                    display: block;
                }
                #info-box {
                    margin-top: 20px;
                    font-size: 18px;
                    font-family: sans-serif;
                }
            </style>
            <script>
                async function fetchInfo() {
                    try {
                        const res = await fetch('/info');
                        const data = await res.json();
                        document.getElementById('info-box').innerText =
                            `Index: ${data.index}, Episode: ${data.episode}`;
                    } catch (e) {
                        console.error("Failed to fetch info", e);
                    }
                }
        """
        html += f"""\
                setInterval(fetchInfo, {freq});
                window.onload = fetchInfo;
            </script>
        </head>
        <body>
            <div class="stream-container">
        """
        for key in self.frames:
            html += f"""\
            <div class="stream-item">
                <h3>{key}</h3>
                <img src="/stream/{key}" />
            </div>
            """
        html += """
            </div>
            <p id="info-box"><b>Info</b> — Index: N/A, Episode: N/A</p>
        </body>
        </html>
        """
        return html

    def on_update(self, data: DictDataStamped[bytes], info: SampleInfo):
        self.info = info
        for key, value_dict in data.items():
            img_bytes = value_dict["data"]
            # can not use / in the key
            key = key.replace("/", ".").removeprefix(".")
            # may be should use a check_data method and
            # use a warmup stage to check
            assert isinstance(img_bytes, bytes), (
                f"frame must be bytes, but got {type(img_bytes)}"
            )
            self.frames[key] = img_bytes
            self.events[key].set()
        return True

    async def _gen_frames(self, stream_id: str):
        while True:
            await self.events[stream_id].wait()
            self.events[stream_id].clear()
            frame = self.frames[stream_id]
            yield b"".join((self.prefix, frame, self.suffix))

    async def _server_task(self):
        config = Config(app=self.app, **self.config.model_dump())
        server = Server(config)
        await server.serve()

    def shutdown(self) -> bool:
        return True


if __name__ == "__main__":
    import io
    import time

    from PIL import Image

    from airdc.common.visualizers.basis import SampleInfo

    def generate_image_bytes(size=(320, 240), color=(100, 100, 200)) -> bytes:
        img = Image.new("RGB", size, color=color)
        buf = io.BytesIO()
        img.save(buf, format="JPEG")
        return buf.getvalue()

    def run_update_loop(visualizer: FastAPIVisualizer):
        index = 0
        while True:
            round_number = index // 250
            info = SampleInfo(index=index, episode=round_number)
            frame = {
                "/left/camera": generate_image_bytes(),
                "/right/camera": generate_image_bytes(color=(200, 100, 100)),
                "/head/camera": generate_image_bytes(color=(100, 200, 100)),
                "/front/camera": generate_image_bytes(color=(100, 100, 200)),
            }
            visualizer.update(frame, info)
            time.sleep(0.02)
            index += 1

    visualizer = FastAPIVisualizer()
    assert visualizer.configure()
    run_update_loop(visualizer)
