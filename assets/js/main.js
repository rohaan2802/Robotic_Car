"use strict";

const root = document.documentElement;
const theme = document.querySelector("#theme");
let preference;
try { preference = localStorage.getItem("robotic-car-theme"); } catch {}
function showTheme(dark) {
  root.dataset.theme = dark ? "dark" : "light";
  const label = dark ? "Switch to light theme" : "Switch to dark theme";
  theme.setAttribute("aria-label", label);
  theme.title = label;
  theme.setAttribute("aria-pressed", dark);
}
showTheme(preference ? preference === "dark" : matchMedia("(prefers-color-scheme: dark)").matches);
theme.addEventListener("click", () => {
  showTheme(root.dataset.theme !== "dark");
  try { localStorage.setItem("robotic-car-theme", root.dataset.theme); } catch {}
});

const menu = document.querySelector("#menu");
const nav = document.querySelector("#nav-links");
const compact = matchMedia("(max-width: 1050px)");
function setMenu(open) {
  nav.classList.toggle("open", open);
  menu.setAttribute("aria-expanded", open);
  menu.setAttribute("aria-label", open ? "Close menu" : "Open menu");
}
menu.addEventListener("click", () => setMenu(!nav.classList.contains("open")));
nav.addEventListener("click", event => {
  if (compact.matches && event.target.closest("a")) {
    setMenu(false);
    menu.focus({preventScroll: true});
  }
});
document.addEventListener("click", event => {
  if (!event.target.closest("header")) setMenu(false);
});
document.addEventListener("keydown", event => {
  if (event.key === "Escape" && nav.classList.contains("open")) {
    setMenu(false);
    menu.focus();
  }
});
compact.addEventListener("change", () => setMenu(false));

const topButton = document.querySelector("#top-button");
function showTop() { topButton.hidden = scrollY < 500; }
addEventListener("scroll", showTop, {passive: true});
topButton.addEventListener("click", () => {
  document.querySelector(".brand").focus({preventScroll:true});
  scrollTo({top: 0});
});
showTop();

const cards = [...document.querySelectorAll(".shot")];
const search = document.querySelector("#search");
const category = document.querySelector("#category");
const count = document.querySelector("#count");
let visible = cards;
function filter() {
  const term = search.value.trim().toLowerCase();
  visible = cards.filter(card => {
    const match = (category.value === "All" || card.dataset.category === category.value)
      && card.textContent.toLowerCase().includes(term);
    card.hidden = !match;
    return match;
  });
  count.textContent = visible.length ? `${visible.length} figure(s)` : "No matching figures.";
}
search.addEventListener("input", filter);
category.addEventListener("change", filter);

const viewer = document.querySelector("#viewer");
const title = document.querySelector("#viewer-title");
const caption = document.querySelector("#caption");
const imageBox = document.querySelector("#image-box");
const image = imageBox.querySelector("img");
const position = document.querySelector("#position");
const zoom = document.querySelector("#zoom");
const previous = document.querySelector("#previous");
const next = document.querySelector("#next");
let opener, index, touchStart;
function setZoom(active) {
  imageBox.classList.toggle("zoomed", active);
  imageBox.scrollTo(0, 0);
  zoom.setAttribute("aria-pressed", active);
  zoom.textContent = active ? "Zoom out" : "Zoom in";
}
function show(delta) {
  index = (index + delta + visible.length) % visible.length;
  const card = visible[index];
  title.textContent = card.querySelector("strong").textContent;
  caption.textContent = card.querySelector("p").textContent;
  image.alt = title.textContent;
  image.src = card.querySelector("a").href;
  previous.disabled = next.disabled = visible.length < 2;
  position.textContent = `${index + 1} / ${visible.length}`;
  setZoom(false);
}
for (const card of cards) {
  const link = card.querySelector("a");
  link.addEventListener("click", event => {
    if (event.ctrlKey || event.metaKey || event.altKey || event.shiftKey) return;
    event.preventDefault();
    opener = link;
    index = visible.indexOf(card);
    show(0);
    viewer.showModal();
  });
}
document.querySelector("#close").addEventListener("click", () => viewer.close());
viewer.addEventListener("close", () => opener.focus());
previous.addEventListener("click", () => show(-1));
next.addEventListener("click", () => show(1));
zoom.addEventListener("click", () => setZoom(!imageBox.classList.contains("zoomed")));
viewer.addEventListener("keydown", event => {
  if (event.target === imageBox && imageBox.classList.contains("zoomed")) return;
  if (event.key === "ArrowLeft" || event.key === "ArrowRight") {
    event.preventDefault();
    show(event.key === "ArrowLeft" ? -1 : 1);
  }
});
imageBox.addEventListener("touchstart", event => {
  touchStart = event.touches.length === 1 ? event.touches[0] : null;
}, {passive: true});
imageBox.addEventListener("touchcancel", () => touchStart = null);
imageBox.addEventListener("touchend", event => {
  const start = touchStart;
  touchStart = null;
  if (!start || imageBox.classList.contains("zoomed")) return;
  const end = event.changedTouches[0];
  const dx = end.clientX - start.clientX;
  const dy = end.clientY - start.clientY;
  if (Math.abs(dx) > 50 && Math.abs(dx) > 1.3 * Math.abs(dy)) show(dx < 0 ? 1 : -1);
}, {passive: true});

const demoVideo = document.querySelector(".video-player video");
const playDemo = document.querySelector("#play-demo");
playDemo.hidden = false;
demoVideo.addEventListener("play", () => { playDemo.hidden = true; });
playDemo.addEventListener("click", async () => {
  try {
    await demoVideo.play();
    playDemo.hidden = true;
    demoVideo.focus();
  } catch {
    // Native controls remain available if the browser rejects playback.
    playDemo.hidden = true;
    demoVideo.focus();
  }
});