// ===== 관리자 HMI =====
// 1초마다 /api/admin/status를 폴링해서 로봇 연결/조인트/그리퍼/TCP 힘/제조 현황/
// 에러 로그를 갱신하고, 제어 버튼(비상 정지/홈 복귀/그리퍼 개폐/속도 모드 전환/
// 키오스크 잠금/에러 로그 지우기)은 POST로 실행한다.

const POLL_INTERVAL_MS = 1000;
let kioskLocked = false; // 최근 폴링 기준 잠금 상태 (토글 스위치 표시용)
let speedMode = null;    // 최근 폴링 기준 속도 모드 (0=일반/1=감속/null=미확인, 세그먼트 표시용)

// ----- 화면 요소 -----
const connBadge = document.getElementById("conn-badge");
const jointsEl = document.getElementById("joints");
const gripperBadge = document.getElementById("gripper-badge");
const forceGridEl = document.getElementById("force-grid");
const makingStateEl = document.getElementById("making-state");
const makingDetailEl = document.getElementById("making-detail");
const lastJobEl = document.getElementById("last-job");
const motionBadge = document.getElementById("motion-badge");
const kioskBadge = document.getElementById("kiosk-badge");
const errorListEl = document.getElementById("error-list");
const controlMsgEl = document.getElementById("control-msg");
const chkLock = document.getElementById("chk-lock");
const btnGripOpen = document.getElementById("btn-grip-open");
const btnGripClose = document.getElementById("btn-grip-close");
const btnSpeedNormal = document.getElementById("btn-speed-normal");
const btnSpeedReduced = document.getElementById("btn-speed-reduced");

// 조인트 카드 6개를 미리 만들어두고 값만 갱신 (매초 DOM 재생성 방지)
const jointVals = [];
for (let i = 0; i < 6; i++) {
  const div = document.createElement("div");
  div.className = "joint";
  div.innerHTML = `<div class="j-name">J${i + 1}</div><div class="j-val">--</div>`;
  jointsEl.appendChild(div);
  jointVals.push(div.querySelector(".j-val"));
}

// TCP 힘/토크 카드 6개 (Fx,Fy,Fz,Mx,My,Mz) — 조인트와 동일한 방식으로 미리 생성
const FORCE_LABELS = ["Fx", "Fy", "Fz", "Mx", "My", "Mz"];
const forceVals = [];
FORCE_LABELS.forEach((label) => {
  const div = document.createElement("div");
  div.className = "joint";
  div.innerHTML = `<div class="j-name">${label}</div><div class="j-val">--</div>`;
  forceGridEl.appendChild(div);
  forceVals.push(div.querySelector(".j-val"));
});

// unknown = I/O 조회 서비스에서 아직 응답을 못 받은 상태 (드라이버 미기동 등)
const GRIPPER_LABELS = { grip: "파지", release: "열림", unknown: "서버 대기 중" };

// 로봇 모션 추정 — 드라이버가 모션 상태를 토픽으로 주지 않아서, 폴링 간격 사이에
// 조인트 각도가 변했는지로 "동작 중/정지"를 판정한다.
const MOTION_THRESHOLD_DEG = 0.2; // 이 값보다 크게 변한 축이 하나라도 있으면 움직임
const MOTION_HOLD_MS = 2500;      // 마지막 움직임 후 이 시간 동안 "동작 중" 유지 (깜빡임 방지)
let prevJoints = null;
let lastMotionTs = 0;

// ----- 상태 폴링 -----
async function poll() {
  try {
    const res = await fetch("/api/admin/status");
    if (res.status === 401) {
      // 세션 만료(30분) 또는 로그아웃 상태 → 로그인 페이지로
      window.location.href = "/admin";
      return;
    }
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    render(await res.json());
  } catch (e) {
    // 백엔드 자체에 연결이 안 되는 상황 (Flask 다운 등)
    connBadge.className = "badge off";
    connBadge.textContent = "서버 연결 끊김";
  }
}

function render(s) {
  // 로봇 연결 배지
  const connected = s.robot && s.robot.connected;
  connBadge.className = `badge ${connected ? "on" : "off"}`;
  connBadge.textContent = connected ? "로봇 연결됨" : "로봇 연결 끊김";

  // 조인트 각도
  const joints = s.robot.joints_deg || [];
  joints.forEach((v, i) => {
    if (jointVals[i]) jointVals[i].textContent = `${v.toFixed(1)}°`;
  });

  // 로봇 모션 배지 — 직전 폴링값과 비교해서 조인트가 움직였는지 판정
  if (prevJoints && joints.some((v, i) => Math.abs(v - prevJoints[i]) > MOTION_THRESHOLD_DEG)) {
    lastMotionTs = Date.now();
  }
  prevJoints = joints.slice();
  if (!connected) {
    motionBadge.className = "badge off";
    motionBadge.textContent = "--";
  } else if (Date.now() - lastMotionTs < MOTION_HOLD_MS) {
    motionBadge.className = "badge warn";
    motionBadge.textContent = "동작 중";
  } else {
    motionBadge.className = "badge idle";
    motionBadge.textContent = "정지";
  }

  // 그리퍼 상태 — cobot_control의 grip/release DO를 로봇에서 직접 조회한 값.
  // 파지/열림 둘 다 정상 상태라 에러색(빨강)은 쓰지 않는다.
  // 로봇 상태 패널 배지와 제어 패널 세그먼트에 함께 반영.
  const gripper = s.robot.gripper || "unknown";
  gripperBadge.className = `badge ${gripper === "grip" ? "on" : "idle"}`;
  gripperBadge.textContent = GRIPPER_LABELS[gripper] || GRIPPER_LABELS.unknown;
  btnGripOpen.classList.toggle("active", gripper === "release");
  btnGripClose.classList.toggle("active", gripper === "grip");

  // TCP 힘/토크 실측값
  (s.robot.tool_force || []).forEach((v, i) => {
    if (forceVals[i]) forceVals[i].textContent = v.toFixed(2);
  });

  // 속도 모드 세그먼트 — 현재 모드 칸이 켜진다 (모드 미확인이면 둘 다 꺼짐)
  speedMode = s.robot.speed_mode ?? null;
  btnSpeedNormal.classList.toggle("active", speedMode === 0);
  btnSpeedReduced.classList.toggle("active", speedMode === 1);

  // 제조 현황
  const m = s.making || {};
  if (m.active) {
    makingStateEl.className = "making-state active";
    makingStateEl.textContent = "제조 중";
    const plan = (m.plan || []).map((p) => `${p.scent} ${p.shots}샷`).join(", ");
    makingDetailEl.innerHTML =
      `<b>${m.recipe_name || ""}</b><br>${plan}<br>경과 ${m.elapsed_sec ?? 0}초`;
  } else {
    makingStateEl.className = "making-state idle";
    makingStateEl.textContent = "대기 중";
    makingDetailEl.textContent = "";
  }
  lastJobEl.textContent = m.last
    ? `직전 작업: ${m.last.recipe_name} · ${m.last.status === "success" ? "성공" : "실패"} (${m.last.finished_at})`
    : "";

  // 에러 로그
  const errors = s.errors || [];
  if (errors.length === 0) {
    errorListEl.innerHTML = `<li class="empty">기록된 에러가 없습니다.</li>`;
  } else {
    errorListEl.innerHTML = errors
      .map((e) =>
        `<li><span class="t">${e.time}</span><span class="lv ${e.level}">${e.level}</span><span>${e.message}</span></li>`)
      .join("");
  }

  // 키오스크 잠금 상태 — 제조 현황 배지 + 제어 탭 토글 스위치 동기화.
  // 주의: HMI가 아는 건 잠금 플래그뿐이다 (키오스크→HMI 단방향 통신이라
  // 키오스크 PC가 실제로 떠 있는지는 알 수 없음). 그래서 "주문 접수 중" 같은
  // 표현 대신 잠금 상태 그대로만 표시한다.
  kioskLocked = !!s.kiosk_locked;
  kioskBadge.className = `badge ${kioskLocked ? "warn" : "on"}`;
  kioskBadge.textContent = kioskLocked ? "잠금 (점검 중)" : "잠금 해제";
  chkLock.checked = kioskLocked;
}

// ----- 제어 버튼 -----
function showControlMsg(text, ok) {
  controlMsgEl.textContent = text;
  controlMsgEl.className = `control-msg ${ok ? "ok" : "err"}`;
}

async function postControl(url, body) {
  try {
    const res = await fetch(url, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    return await res.json();
  } catch (e) {
    return { status: "error", message: "서버와 통신하지 못했습니다." };
  }
}

// 비상 정지 — 긴급 상황용이므로 확인창 없이 즉시 실행
document.getElementById("btn-estop").addEventListener("click", async () => {
  showControlMsg("비상 정지 전송 중...", true);
  const r = await postControl("/api/admin/estop");
  showControlMsg(r.message || "", r.status === "success");
});

// 홈 복귀 — 이동이 끝나야 응답이 오므로(SYNC) 완료까지 버튼 잠금
document.getElementById("btn-home").addEventListener("click", async (e) => {
  const btn = e.currentTarget;
  btn.disabled = true;
  showControlMsg("홈 복귀 중... (완료까지 대기)", true);
  const r = await postControl("/api/admin/home");
  showControlMsg(r.message || "", r.status === "success");
  btn.disabled = false;
});

// 키오스크 잠금/해제 토글 스위치
chkLock.addEventListener("change", async () => {
  const r = await postControl("/api/admin/lock", { locked: chkLock.checked });
  showControlMsg(r.locked ? "키오스크를 잠갔습니다 (점검 중 표시)" : "키오스크 잠금을 해제했습니다", true);
  poll(); // 스위치 표시 즉시 갱신
});

// 그리퍼 수동 열기/닫기 — 팔 모션 없이 I/O만 바꾼다
btnGripOpen.addEventListener("click", async () => {
  const r = await postControl("/api/admin/gripper", { action: "release" });
  showControlMsg(r.message || "", r.status === "success");
  poll();
});
btnGripClose.addEventListener("click", async () => {
  const r = await postControl("/api/admin/gripper", { action: "grip" });
  showControlMsg(r.message || "", r.status === "success");
  poll();
});

// 속도 모드 세그먼트 — 원하는 모드를 직접 선택 (현재 모드를 몰라도 동작)
btnSpeedNormal.addEventListener("click", async () => {
  const r = await postControl("/api/admin/speed_mode", { mode: "normal" });
  showControlMsg(r.message || "", r.status === "success");
  poll();
});
btnSpeedReduced.addEventListener("click", async () => {
  const r = await postControl("/api/admin/speed_mode", { mode: "reduced" });
  showControlMsg(r.message || "", r.status === "success");
  poll();
});

// 에러 로그 지우기
document.getElementById("btn-clear-errors").addEventListener("click", async () => {
  await postControl("/api/admin/clear_errors");
  poll();
});

// 로그아웃 → 세션 지우고 로그인 페이지로
document.getElementById("btn-logout").addEventListener("click", async () => {
  await postControl("/api/admin/logout");
  window.location.href = "/admin";
});

// ----- 시계 + 폴링 시작 -----
setInterval(() => {
  document.getElementById("clock").textContent = new Date().toLocaleTimeString("ko-KR");
}, 1000);
poll();
setInterval(poll, POLL_INTERVAL_MS);
