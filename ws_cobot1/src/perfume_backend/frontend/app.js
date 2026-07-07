// ===== 향료 데이터 (탭2·탭3 UI용) =====
const SCENTS = {
  top: ["Citrus", "Green"],
  middle: ["Floral", "Woody"],
  base: ["Musk", "Amber"],
};
const ALL_SCENTS = [...SCENTS.top, ...SCENTS.middle, ...SCENTS.base];
const FREE_SLOT_COUNT = 6;
const FREE_MAX_SHOTS = 6; // 1샷 = 향료 1회 토출

// 추천 카드용 아이콘 (맑은 날 / 흐린 날 / 밤)
const CARD_ICONS = [
  `<svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="4.5" fill="none" stroke="currentColor" stroke-width="1.8"/>
     <g stroke="currentColor" stroke-width="1.8" stroke-linecap="round">
       <line x1="12" y1="2.5" x2="12" y2="5"/><line x1="12" y1="19" x2="12" y2="21.5"/>
       <line x1="2.5" y1="12" x2="5" y2="12"/><line x1="19" y1="12" x2="21.5" y2="12"/>
       <line x1="5.3" y1="5.3" x2="7" y2="7"/><line x1="17" y1="17" x2="18.7" y2="18.7"/>
       <line x1="18.7" y1="5.3" x2="17" y2="7"/><line x1="5.3" y1="18.7" x2="7" y2="17"/>
     </g></svg>`,
  `<svg viewBox="0 0 24 24"><g fill="currentColor">
       <ellipse cx="12" cy="6" rx="2.6" ry="3.6" opacity=".9"/>
       <ellipse cx="12" cy="18" rx="2.6" ry="3.6" opacity=".9"/>
       <ellipse cx="6" cy="12" rx="3.6" ry="2.6" opacity=".9"/>
       <ellipse cx="18" cy="12" rx="3.6" ry="2.6" opacity=".9"/>
       <circle cx="12" cy="12" r="2.4"/>
     </g></svg>`,
  `<svg viewBox="0 0 24 24"><path d="M20 14.5 A 8.5 8.5 0 1 1 9.5 4 A 7 7 0 0 0 20 14.5 z" fill="currentColor"/>
     <circle cx="18" cy="5.5" r="1.2" fill="currentColor"/></svg>`,
];

let doneTimer = null;

// HTML 삽입용 이스케이프 (DB 텍스트를 innerHTML에 넣을 때 사용)
function esc(str) {
  const div = document.createElement("div");
  div.textContent = str;
  return div.innerHTML;
}

// ===== 화면 전환 =====
function showScreen(id) {
  document.querySelectorAll(".screen").forEach((s) => s.classList.remove("active"));
  document.getElementById(id).classList.add("active");
  resetIdleTimer();
}

// ===== 토스트 알림 (alert() 팝업 대신 화면 안에서 자연스럽게 보여주는 안내) =====
let toastTimer = null;
function showToast(message, type = "error") {
  const toast = document.getElementById("toast");
  toast.textContent = message;
  toast.className = `toast show ${type}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => toast.classList.remove("show"), 3200);
}

// ===== 자동 초기화(idle timeout) =====
// 키오스크 특성상 손님이 중간에 그냥 가버리는 경우를 대비해, 일정 시간 터치가
// 없으면 자동으로 시작 화면으로 되돌린다. 단, 시작 화면(이미 홈)과 제조중
// 화면(로봇이 실제로 끝날 때까지는 화면을 강제로 바꾸면 안 됨)에서는 걸지 않는다.
const IDLE_TIMEOUT_MS = 60000;
let idleTimer = null;
function resetIdleTimer() {
  clearTimeout(idleTimer);
  const current = document.querySelector(".screen.active");
  if (!current) return;
  // 시작(이미 홈)·제조중(강제 전환 금지)·점검중(잠금 해제로만 복귀) 화면은 제외
  if (["screen-start", "screen-making", "screen-maintenance"].includes(current.id)) return;
  idleTimer = setTimeout(goHome, IDLE_TIMEOUT_MS);
}
["touchstart", "mousedown", "keydown"].forEach((evt) =>
  document.addEventListener(evt, resetIdleTimer, { passive: true })
);

// ===== 관리자 숨김 진입 =====
// 시작 화면 오른쪽 위 모서리(admin-hotspot)를 3초 길게 누르면 관리자 페이지로.
// 눈에 보이는 버튼을 두면 손님이 눌러볼 수 있어서(관리자 페이지엔 비상 정지가
// 있음) 직원만 아는 숨김 제스처로 만들었다.
const ADMIN_HOLD_MS = 3000;
let adminHoldTimer = null;
const adminHotspot = document.getElementById("admin-hotspot");
adminHotspot.addEventListener("pointerdown", () => {
  adminHoldTimer = setTimeout(() => (window.location.href = "/admin"), ADMIN_HOLD_MS);
});
["pointerup", "pointerleave", "pointercancel"].forEach((evt) =>
  adminHotspot.addEventListener(evt, () => clearTimeout(adminHoldTimer))
);
// 짧게 탭했을 때 시작 화면의 "터치하여 시작"이 발동하지 않도록 클릭 전파 차단
adminHotspot.addEventListener("click", (e) => e.stopPropagation());

// ===== 점검 중(관리자 잠금) 처리 =====
// 관리자가 HMI에서 키오스크를 잠그면 점검 화면을 띄우고,
// 5초마다 잠금 해제 여부를 확인해서 풀리면 시작 화면으로 복귀한다.
let maintenanceTimer = null;
function enterMaintenance() {
  showScreen("screen-maintenance");
  clearInterval(maintenanceTimer);
  maintenanceTimer = setInterval(async () => {
    try {
      const res = await fetch("/api/kiosk_status");
      const s = await res.json();
      if (!s.locked) {
        clearInterval(maintenanceTimer);
        goHome();
      }
    } catch (e) { /* 서버 연결 실패 시 다음 주기에 재시도 */ }
  }, 5000);
}

// ===== 시작 화면: 터치 시 메인으로 (잠금 상태면 점검 화면으로) =====
document.getElementById("screen-start").addEventListener("click", async () => {
  try {
    const res = await fetch("/api/kiosk_status");
    const s = await res.json();
    if (s.locked) {
      enterMaintenance();
      return;
    }
  } catch (e) { /* 상태 확인 실패 시엔 일단 진행 (주문 시점에 한 번 더 걸러짐) */ }
  loadRecipes();
  showScreen("screen-main");
});

// ===== 탭 전환 =====
document.querySelectorAll(".tab").forEach((tab) => {
  tab.addEventListener("click", () => {
    document.querySelectorAll(".tab").forEach((t) => t.classList.remove("active"));
    document.querySelectorAll(".tab-panel").forEach((p) => p.classList.remove("active"));
    tab.classList.add("active");
    document.getElementById(tab.dataset.tab).classList.add("active");
  });
});

// ===== Tab 1: 추천 카드 로드 =====
async function loadRecipes() {
  const container = document.getElementById("recipe-cards");
  container.innerHTML = "<p class='hint'>불러오는 중...</p>";
  try {
    const res = await fetch("/api/recipes");
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const recipes = await res.json();
    container.innerHTML = "";
    recipes.forEach((r, i) => {
      const card = document.createElement("div");
      card.className = "card";
      card.innerHTML = `
        <div class="card-top">
          <div class="card-icon icon-${i % 3}">${CARD_ICONS[i % 3]}</div>
          <div>
            <h3>${esc(r.recipe_name)}</h3>
            <p class="desc">${esc(r.description || "")}</p>
          </div>
        </div>
        <div class="ratios">
          <span class="chip-top">${esc(r.top_scent)} ${r.top_ratio}%</span>
          <span class="chip-mid">${esc(r.mid_scent)} ${r.mid_ratio}%</span>
          <span class="chip-base">${esc(r.base_scent)} ${r.base_ratio}%</span>
        </div>
        <button class="make-btn">제조하기</button>`;
      card.querySelector("button").addEventListener("click", () =>
        submitPerfume({ mode: "recommend", recipe_id: r.id })
      );
      container.appendChild(card);
    });
  } catch (e) {
    container.innerHTML = "<p class='hint'>레시피를 불러오지 못했습니다.</p>";
  }
}

// ===== Tab 2: 나만의 조합 UI 생성 =====
function buildCustomTab() {
  const wrap = document.getElementById("custom-layers");
  const labels = { top: "Top", middle: "Middle", base: "Base" };
  wrap.innerHTML = "";
  Object.keys(SCENTS).forEach((layer) => {
    const div = document.createElement("div");
    div.className = "layer";
    div.innerHTML = `<h4><span class="dot dot-${layer}"></span>${labels[layer]} Note</h4>`;
    SCENTS[layer].forEach((scent) => {
      div.innerHTML += `<label><input type="checkbox" data-layer="${layer}"
        value="${scent}" /> ${scent}</label>`;
    });
    wrap.appendChild(div);
  });
}

document.querySelector('[data-mode="custom"]').addEventListener("click", () => {
  const selections = { top: [], middle: [], base: [] };
  document.querySelectorAll("#custom-layers input:checked").forEach((cb) => {
    selections[cb.dataset.layer].push(cb.value);
  });
  const total = selections.top.length + selections.middle.length + selections.base.length;
  if (total === 0) {
    showToast("향료를 하나 이상 선택해주세요.");
    return;
  }
  submitPerfume({ mode: "custom", selections });
});

// ===== Tab 3: 내맘대로 슬롯 UI 생성 =====
function buildFreeTab() {
  const wrap = document.getElementById("free-slots");
  wrap.innerHTML = "";
  const scentOpts =
    `<option value="">향료 선택</option>` +
    ALL_SCENTS.map((s) => `<option value="${s}">${s}</option>`).join("");
  const shotOpts = [0, 1, 2, 3, 4, 5, 6]
    .map((n) => `<option value="${n}">${n}샷</option>`)
    .join("");
  for (let i = 0; i < FREE_SLOT_COUNT; i++) {
    const slot = document.createElement("div");
    slot.className = "slot";
    slot.innerHTML = `
      <div class="slot-badge">${i + 1}</div>
      <div class="slot-fields">
        <label class="field">
          <span class="field-label">향료</span>
          <select class="free-scent">${scentOpts}</select>
        </label>
        <label class="field">
          <span class="field-label">샷</span>
          <select class="free-shots">${shotOpts}</select>
        </label>
      </div>`;
    wrap.appendChild(slot);
  }
  wrap.querySelectorAll(".free-shots").forEach((sel) =>
    sel.addEventListener("change", updateFreeTotal)
  );
}

function updateFreeTotal() {
  let total = 0;
  document.querySelectorAll("#free-slots .free-shots").forEach((sel) => {
    total += parseInt(sel.value, 10);
  });
  document.getElementById("free-total").textContent = total;
  const over = total > FREE_MAX_SHOTS;
  document.getElementById("free-counter").classList.toggle("over", over);
  document.getElementById("free-error").hidden = !over;
  return total;
}

document.querySelector('[data-mode="free"]').addEventListener("click", () => {
  const slots = [];
  document.querySelectorAll("#free-slots .slot").forEach((slot) => {
    const scent = slot.querySelector(".free-scent").value;
    const shots = parseInt(slot.querySelector(".free-shots").value, 10);
    if (scent && shots > 0) slots.push({ scent, shots });
  });
  if (slots.length === 0) {
    showToast("향료와 토출 횟수(샷)를 선택해주세요.");
    return;
  }
  const total = slots.reduce((sum, s) => sum + s.shots, 0);
  if (total > FREE_MAX_SHOTS) {
    showToast(`총 ${FREE_MAX_SHOTS}샷까지만 담을 수 있습니다. (현재 ${total}샷)`);
    return;
  }
  submitPerfume({ mode: "free", slots });
});

// ===== 제조 요청 (fetch POST) =====
// 아래 fetch는 백엔드가 실제 로봇(M0609)의 제조가 끝날 때까지 응답을 미루도록
// 되어 있다(backend/robot_control.py 참고). 그래서 이 함수는 별도 폴링 없이도
// "fetch가 끝날 때 = 로봇이 실제로 제조를 끝냈을 때"가 되어, 그 사이에는
// screen-making('제조중' 화면)이 그대로 유지된다.
let isSubmitting = false; // "제조하기" 연타로 인한 중복 요청 방지 플래그

async function submitPerfume(payload) {
  if (isSubmitting) return;
  isSubmitting = true;
  showScreen("screen-making");
  try {
    const res = await fetch("/api/make_perfume", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const result = await res.json();
    if (res.ok && result.status === "success") {
      showDone(result);
    } else if (res.status === 503) {
      enterMaintenance(); // 주문 도중 관리자가 키오스크를 잠근 경우
    } else {
      showScreen("screen-main");
      showToast(result.message || "제조에 실패했습니다.");
    }
  } catch (e) {
    showScreen("screen-main");
    showToast("서버와 통신하지 못했습니다.");
  } finally {
    isSubmitting = false;
  }
}

// ===== 완료 화면 + 10초 자동 리다이렉트 =====
function showDone(result) {
  const summary = (result.plan || [])
    .map((p) => `${p.scent} ${p.shots}샷`)
    .join(", ");
  document.getElementById("done-summary").textContent =
    `${result.recipe_name} (총 ${result.total_shots}샷)${summary ? " · " + summary : ""}`;
  showScreen("screen-done");

  clearTimeout(doneTimer);
  doneTimer = setTimeout(goHome, 10000);
}

function goHome() {
  clearTimeout(doneTimer);
  // 상태 초기화 — 다음 손님이 이전 손님의 흔적을 보지 않도록 전부 되돌린다
  document.querySelectorAll("#custom-layers input:checked").forEach((cb) => (cb.checked = false));
  buildFreeTab();
  updateFreeTotal();
  // 탭도 첫 번째(오늘의 추천)로 되돌리고 각 패널 스크롤을 맨 위로
  document.querySelectorAll(".tab").forEach((t) =>
    t.classList.toggle("active", t.dataset.tab === "tab-recommend")
  );
  document.querySelectorAll(".tab-panel").forEach((p) => {
    p.classList.toggle("active", p.id === "tab-recommend");
    p.scrollTop = 0;
  });
  showScreen("screen-start");
}

document.getElementById("home-btn").addEventListener("click", goHome);
document.getElementById("header-home").addEventListener("click", goHome);

// ===== 초기화 =====
buildCustomTab();
buildFreeTab();
updateFreeTotal();

// 제조중 화면에 시작 화면 로봇 애니메이션을 복제해 재사용 (CSS가 속도를 올림)
document
  .getElementById("making-anim")
  .appendChild(document.querySelector(".hero-anim svg").cloneNode(true));
