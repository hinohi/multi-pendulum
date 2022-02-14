import {App} from "multi-pendulum";

const canvas = document.getElementById("canvas") as HTMLCanvasElement | null;
if (!canvas) {
  throw new Error("No 'canvas'");
}

const app = new App(canvas);
const e0 = app.potential_energy() + app.kinetic_energy();

function step(timestamp: DOMHighResTimeStamp): void {
  const energyDiv = document.getElementById("energy") as HTMLDivElement | null;
  if (!energyDiv) {
    throw new Error("No 'energy'");
  }
  app.tick(timestamp);
  const pe = app.potential_energy();
  const ke = app.kinetic_energy();
  const unit = app.unit_energy();
  energyDiv.innerText = `U = ${pe.toExponential(2)} (${(pe / unit).toExponential(2)})
T = ${ke.toExponential(2)} (${(pe / unit).toExponential(2)})
ΔE = E - E0 = ${(pe + ke - e0).toExponential(2)} (${((pe + ke - e0) / unit).toExponential(2)})`;
  window.requestAnimationFrame(step);
}

window.requestAnimationFrame(step);
