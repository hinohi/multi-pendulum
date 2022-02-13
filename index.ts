import {App} from "multi-pendulum";

const canvas = document.getElementById("canvas") as HTMLCanvasElement | null;
if (!canvas) {
  throw new Error("No 'canvas'");
}

const app = new App(canvas);

function step(timestamp: DOMHighResTimeStamp): void {
  app.tick(timestamp);
  window.requestAnimationFrame(step);
}

window.requestAnimationFrame(step);
