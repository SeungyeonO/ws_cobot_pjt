// ===== 관리자 HMI =====
// 1초마다 /api/admin/status를 폴링해서 로봇 연결/조인트/제조 현황/에러 로그를
// 갱신하고, 제어 버튼(비상 정지/홈 복귀/키오스크 잠금)은 POST로 실행한다.

const POLL_INTERVAL_MS = 1000;
let kioskLocked = false; // 최근 폴링 기준 잠금 상태 (토글 버튼 표시용)

// ----- 화면 요소 -----
const connBadge = document.getElementById("conn-badge");
const jointsEl = document.getElementById("joints");
const makingStateEl = document.getElementById("making-state");
const makingDetailEl = document.getElementById("making-detail");
const lastJobEl = document.getElementById("last-job");
const errorListEl = document.getElementById("error-list");
const controlMsgEl = document.getElementById("control-msg");
const btnLock = document.getElementById("btn-lock");

// 조인트 카드 6개를 미리 만들어두고 값만 갱신 (매초 DOM 재생성 방지)
const jointVals = [];
for (let i = 0; i < 6; i++) {
  const div = document.createElement("div");
  div.className = "joint";
  div.innerHTML = `<div class="j-name">J${i + 1}</div><div class="j-val">--</div>`;
  jointsEl.appendChild(div);
  jointVals.push(div.querySelector(".j-val"));
}

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
  (s.robot.joints_deg || []).forEach((v, i) => {
    if (jointVals[i]) jointVals[i].textContent = `${v.toFixed(1)}°`;
  });

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

  // 키오스크 잠금 토글 버튼 상태
  kioskLocked = !!s.kiosk_locked;
  btnLock.textContent = kioskLocked ? "잠금 해제" : "키오스크 잠금";
  btnLock.classList.toggle("locked", kioskLocked);
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

// 키오스크 잠금/해제 토글
btnLock.addEventListener("click", async () => {
  const r = await postControl("/api/admin/lock", { locked: !kioskLocked });
  showControlMsg(r.locked ? "키오스크를 잠갔습니다 (점검 중 표시)" : "키오스크 잠금을 해제했습니다", true);
  poll(); // 버튼 표시 즉시 갱신
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
