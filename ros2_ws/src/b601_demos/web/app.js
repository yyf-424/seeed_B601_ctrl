"use strict";

const element = (id) => document.getElementById(id);
const group = element("group");
const inputSource = element("input-source");
const sourceLabels = new Map([["web", "网页关节"], ["web_ik", "网页 TCP"]]);
const sessions = new Map();
const rows = new Map();
const edited = new Set();
let snapshot = { feedback: {}, limits: {} };
let busy = false;
let robotView;

function controllers() {
  return snapshot.status?.controllers || {};
}

function refreshRobotView() {
  if (!robotView) return;
  const target = {};
  for (const state of Object.values(controllers())) {
    if (!state.holding_after_loss) Object.assign(target, state.last_forwarded?.position);
  }
  let label = "最近下发目标";
  for (const name of edited) {
    const input = rows.get(name).number;
    if (Number.isFinite(input.valueAsNumber) && input.checkValidity()) {
      target[name] = input.valueAsNumber;
      label = "待发送预览";
    }
  }
  const armJoints = group.value === "b601_mit_arm" ? controllers()[group.value]?.joints : [];
  robotView.update({
    feedback: snapshot.feedback, target, label, online: !!snapshot.status,
    armJoints, limits: snapshot.limits,
  });
}

function applyTcpDraft(positions) {
  for (const [name, value] of Object.entries(positions)) {
    const row = rows.get(name);
    if (!row) continue;
    const { min, max } = snapshot.limits[name];
    setValue(row, Math.max(min, Math.min(max, Number(value.toFixed(5)))));
    edited.add(name);
  }
  refresh();
}

function notice(message, error = false) {
  element("notice").textContent = message;
  element("notice").dataset.error = String(error);
}

function setValue(row, value) {
  row.number.value = value ?? "";
  if (value !== undefined) row.slider.value = value;
}

function makeRows() {
  rows.clear();
  edited.clear();
  const list = element("joints");
  list.replaceChildren();
  for (const name of controllers()[group.value]?.joints || []) {
    const row = document.createElement("div");
    row.className = "joint";
    const title = document.createElement("span");
    title.className = "joint-name";
    title.textContent = name;
    const measured = document.createElement("span");
    measured.className = "measured";
    const slider = document.createElement("input");
    const number = document.createElement("input");
    const unit = name.startsWith("finger_") ? "m" : "rad";
    slider.type = "range";
    slider.step = unit === "m" ? "0.0001" : "0.001";
    number.type = "number";
    number.step = "any";
    for (const input of [slider, number]) {
      input.min = snapshot.limits[name].min;
      input.max = snapshot.limits[name].max;
    }
    slider.setAttribute("aria-label", `${name} 目标滑块 ${unit}`);
    number.setAttribute("aria-label", `${name} 目标 ${unit}`);
    slider.addEventListener("input", () => {
      number.value = slider.value;
      edited.add(name);
      refresh();
    });
    number.addEventListener("input", () => {
      slider.value = number.value;
      edited.add(name);
      refresh();
    });
    const send = document.createElement("button");
    send.className = "quiet";
    send.textContent = "发送";
    send.setAttribute("aria-label", `发送 ${name} 目标`);
    send.addEventListener("click", () => mutate(() => sendTargets([name])));
    row.append(title, measured, slider, number, send);
    list.append(row);
    rows.set(name, { measured, slider, number, send, unit });
  }
}

function refresh() {
  const state = controllers()[group.value];
  const session = sessions.get(group.value);
  const allowed = !busy && state?.session === session?.publicId && !!session;
  const connection = element("connection");
  connection.dataset.connected = String(!!state && snapshot.feedback_age_ms !== null);
  connection.textContent = state ? "仲裁在线" : "等待仲裁节点";
  element("source").textContent = sourceLabels.get(state?.source) || state?.source || "无";
  element("age").textContent = snapshot.feedback_age_ms === null || snapshot.feedback_age_ms === undefined
    ? "—" : `${snapshot.feedback_age_ms} ms`;
  const forwarded = state?.last_forwarded;
  element("applied").textContent = state?.holding_after_loss ? "受控持位"
    : !forwarded ? "—" : state.last_applied_id === forwarded.dispatch_id ? "已采纳" : "等待采纳";
  element("acquire").disabled = busy || !state || !!state.source || !!session;
  element("release").disabled = busy || !state?.session;
  const supportsTcp = group.value === "b601_mit_arm";
  inputSource.options[1].disabled = !supportsTcp;
  if (!supportsTcp && inputSource.value === "web_ik") inputSource.value = "web";
  inputSource.disabled = busy || !state || !!state.source || !!session;
  element("sync").disabled = !state || !Object.keys(snapshot.feedback).length;
  element("send-all").disabled = !allowed || !edited.size;
  group.disabled = busy || !state;
  for (const [name, row] of rows) {
    const value = snapshot.feedback[name];
    row.measured.textContent = value === undefined ? "—" : `${value.toFixed(4)} ${row.unit}`;
    if (!edited.has(name) && document.activeElement !== row.number && document.activeElement !== row.slider) {
      const desired = !state?.holding_after_loss && forwarded?.session === state?.session
        ? forwarded?.position[name] : undefined;
      setValue(row, desired ?? value);
    }
    row.send.disabled = !allowed || !edited.has(name);
  }
  refreshRobotView();
}

async function poll() {
  try {
    const response = await fetch("/api/state");
    if (!response.ok) throw new Error("状态读取失败");
    snapshot = await response.json();
    const names = Object.keys(controllers());
    const current = group.value;
    for (const [name, session] of sessions) {
      const active = controllers()[name]?.session;
      if (active === session.publicId) session.observed = true;
      else if (session.observed) sessions.delete(name);
    }
    if (names.join() !== [...group.options].map((item) => item.value).join()) {
      group.replaceChildren(...names.map((name) => new Option(
        name === "b601_mit_arm" ? "手臂六轴" : name === "b601_mit_gripper" ? "夹爪" : name, name)));
      if (names.includes(current)) group.value = current;
      makeRows();
    }
    refresh();
  } catch (error) {
    snapshot.status = null;
    refresh();
    notice(error.message, true);
  } finally {
    window.setTimeout(poll, 100);
  }
}

async function api(path, payload) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload),
  });
  const result = await response.json();
  if (!response.ok) {
    const error = new Error(result.message);
    error.status = response.status;
    throw error;
  }
  return result;
}

function renewSessions() {
  for (const [name, session] of sessions) {
    if (session.renewing) continue;
    session.renewing = true;
    api("/api/heartbeat", { session_id: session.id }).catch((error) => {
      if (error.status === 409 && sessions.get(name) === session) {
        sessions.delete(name);
        notice("网页控制权已失效，请重新接管。", true);
        refresh();
      }
    }).finally(() => { session.renewing = false; });
  }
  window.setTimeout(renewSessions, snapshot.heartbeat_period_ms || 100);
}

async function mutate(task) {
  if (busy) return;
  busy = true;
  refresh();
  try {
    await task();
  } catch (error) {
    notice(error.message, true);
  } finally {
    busy = false;
    refresh();
  }
}

async function sendTargets(names) {
  const session = sessions.get(group.value);
  const positions = Object.fromEntries(names.map((name) => {
    const input = rows.get(name).number;
    if (!input.value || !input.checkValidity()) {
      input.reportValidity();
      throw new Error(`${name} 超出关节范围，或未填写数值。`);
    }
    return [name, input.valueAsNumber];
  }));
  const reply = await api("/api/target", {
    session_id: session.id, sequence: ++session.sequence, positions,
  });
  for (const name of names) edited.delete(name);
  notice(`目标已转发，编号 ${reply.dispatch_id}。`);
}

group.addEventListener("change", () => { makeRows(); refresh(); });
element("acquire").addEventListener("click", () => mutate(async () => {
  const source = inputSource.value;
  const reply = await api("/api/select", { controller: group.value, source });
  sessions.set(group.value, { id: reply.session_id, publicId: reply.session, sequence: 0 });
  notice(`${sourceLabels.get(source)}已取得控制权。`);
}));
element("release").addEventListener("click", () => mutate(async () => {
  const controller = group.value;
  const state = controllers()[controller];
  const session = sessions.get(controller);
  if (session?.publicId === state.session) {
    await api("/api/release", { session_id: session.id });
    sessions.delete(controller);
  } else {
    const groups = Object.entries(controllers()).filter(([, route]) => route.session === state.session)
      .map(([name]) => name === "b601_mit_arm" ? "手臂六轴" : name === "b601_mit_gripper" ? "夹爪" : name);
    if (!window.confirm(`释放 ${sourceLabels.get(state.source) || state.source} 的 ${groups.join("、")} 控制权？`)) return;
    await api("/api/revoke", { controller, session: state.session });
  }
  notice("已释放控制权。");
}));
element("sync").addEventListener("click", () => {
  for (const [name, row] of rows) {
    if (snapshot.feedback[name] === undefined) continue;
    setValue(row, snapshot.feedback[name]);
    edited.add(name);
  }
  refresh();
});
element("send-all").addEventListener("click", () => mutate(() => sendTargets([...edited])));
window.addEventListener("pagehide", () => { sessions.clear(); });
import("/robot-view.js").then(({ createRobotView }) => {
  robotView = createRobotView(element("robot-view"), applyTcpDraft);
  refreshRobotView();
}).catch((error) => {
  element("model-status").textContent = `三维渲染不可用：${error.message}`;
});
poll();
renewSessions();
