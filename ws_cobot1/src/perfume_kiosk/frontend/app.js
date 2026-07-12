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
// 눈에 보이는 버튼을 두면 손님이 눌러볼 수 있어서(관리자 페이지엔 로봇 정지가
// 있음) 직원만 아는 숨김 제스처로 만들었다.
// HMI(관리자 페이지)는 로봇 제어 PC에서 별도로 실행되므로(perfume_hmi 패키지),
// 배포 시 아래 주소를 실제 로봇 제어 PC의 IP로 바꿔야 한다.
const ADMIN_URL = "http://<HMI-PC-IP>:5000/admin";
const ADMIN_HOLD_MS = 3000;
let adminHoldTimer = null;
const adminHotspot = document.getElementById("admin-hotspot");
adminHotspot.addEventListener("pointerdown", () => {
  adminHoldTimer = setTimeout(() => (window.location.href = ADMIN_URL), ADMIN_HOLD_MS);
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
// 향료별 표기·설명·라인 아이콘 (reference 디자인). value(영문명)는 백엔드 규약이라 그대로 둔다.
const SCENT_META = {
  Citrus: {
    name: "시트러스",
    desc: "자몽, 귤, 오렌지 등의 과일에서 추출한<br>상큼하고 새콤한 향",
    icons: [
      { label: "자몽", svg: `<svg viewBox="0 0 64 64"><circle cx="31" cy="34" r="19"></circle><path d="M31 15 C36 23, 36 45, 31 53"></path><path d="M18 25 C26 29, 38 29, 45 25"></path><path d="M17 43 C26 38, 38 38, 46 43"></path><path d="M36 15 C42 9, 49 11, 51 17 C45 19, 40 19, 36 15"></path></svg>` },
      { label: "귤", svg: `<svg viewBox="0 0 64 64"><path d="M16 35 C16 22, 26 14, 38 17 C49 20, 55 31, 50 43 C45 55, 28 58, 20 49 C17 46, 16 41, 16 35Z"></path><path d="M32 17 C35 12, 42 9, 48 13"></path><path d="M34 18 C31 25, 28 34, 30 50"></path><path d="M22 32 C30 35, 42 34, 50 30"></path></svg>` },
      { label: "오렌지", svg: `<svg viewBox="0 0 64 64"><circle cx="32" cy="35" r="20"></circle><circle cx="32" cy="35" r="7"></circle><path d="M32 15 L32 55"></path><path d="M12 35 L52 35"></path><path d="M18 21 L46 49"></path><path d="M46 21 L18 49"></path><path d="M38 13 C42 8, 49 9, 52 15 C46 18, 42 18, 38 13"></path></svg>` },
    ],
  },
  Green: {
    name: "그린",
    desc: "갓 베어낸 풀, 싱그러운 나뭇잎에서 나는<br>싱그럽고 산뜻한 자연의 향",
    icons: [
      { label: "풀", svg: `<svg viewBox="0 0 64 64"><path d="M18 52 C21 39, 22 29, 20 17"></path><path d="M31 52 C32 38, 31 27, 29 14"></path><path d="M44 52 C42 39, 43 28, 47 16"></path><path d="M14 52 H50"></path></svg>` },
      { label: "나뭇잎", svg: `<svg viewBox="0 0 64 64"><path d="M15 41 C22 18, 43 12, 53 16 C52 35, 39 49, 18 48 C17 46, 16 44, 15 41Z"></path><path d="M18 47 C28 38, 37 30, 50 17"></path><path d="M29 38 C29 32, 27 27, 24 23"></path><path d="M39 29 C35 27, 31 25, 26 25"></path></svg>` },
      { label: "이끼", svg: `<svg viewBox="0 0 64 64"><path d="M15 46 C18 34, 30 31, 36 39 C41 33, 52 36, 52 47 Z"></path><path d="M20 44 C22 39, 27 37, 30 40"></path><path d="M35 44 C38 39, 44 39, 47 43"></path><path d="M25 32 C25 26, 28 22, 32 19"></path><path d="M39 34 C41 28, 45 24, 50 23"></path></svg>` },
    ],
  },
  Floral: {
    name: "플로럴",
    desc: "장미, 자스민, 라일락 등에서 추출한<br>우아하고 생기있는 꽃 향",
    icons: [
      { label: "장미", svg: `<svg viewBox="0 0 64 64"><path d="M32 34 C22 34, 17 26, 22 19 C26 13, 36 15, 36 24"></path><path d="M32 34 C43 33, 48 25, 43 18 C39 12, 30 15, 29 24"></path><path d="M32 34 C27 28, 29 20, 36 18"></path><path d="M32 34 V54"></path><path d="M32 45 C25 42, 20 43, 16 48"></path><path d="M32 46 C39 42, 44 43, 48 48"></path></svg>` },
      { label: "자스민", svg: `<svg viewBox="0 0 64 64"><circle cx="32" cy="31" r="6"></circle><path d="M32 12 C38 20, 38 26, 32 31 C26 26, 26 20, 32 12Z"></path><path d="M32 50 C26 42, 26 36, 32 31 C38 36, 38 42, 32 50Z"></path><path d="M13 31 C21 25, 27 25, 32 31 C27 37, 21 37, 13 31Z"></path><path d="M51 31 C43 25, 37 25, 32 31 C37 37, 43 37, 51 31Z"></path></svg>` },
      { label: "라일락", svg: `<svg viewBox="0 0 64 64"><path d="M25 54 C27 40, 28 28, 27 14"></path><path d="M39 54 C37 40, 37 27, 39 13"></path><circle cx="27" cy="16" r="5"></circle><circle cx="22" cy="25" r="5"></circle><circle cx="31" cy="26" r="5"></circle><circle cx="39" cy="15" r="5"></circle><circle cx="44" cy="24" r="5"></circle><circle cx="36" cy="28" r="5"></circle></svg>` },
    ],
  },
  Woody: {
    name: "우디",
    desc: "숲 속의 나무, 흙, 이끼 등을 연상시키는<br>깊이감 있는 향",
    icons: [
      { label: "나무", svg: `<svg viewBox="0 0 64 64"><path d="M32 54 V24"></path><path d="M32 34 L21 24"></path><path d="M32 38 L44 27"></path><path d="M19 25 C14 18, 17 10, 25 11 C27 5, 37 5, 40 12 C48 12, 52 20, 47 27 C39 26, 27 27, 19 25Z"></path><path d="M20 54 H45"></path></svg>` },
      { label: "흙", svg: `<svg viewBox="0 0 64 64"><path d="M14 45 C20 37, 27 35, 34 40 C41 33, 51 37, 54 47"></path><path d="M12 50 H54"></path><circle cx="23" cy="42" r="2"></circle><circle cx="34" cy="45" r="2"></circle><circle cx="45" cy="42" r="2"></circle><path d="M22 31 C27 27, 33 27, 38 31"></path></svg>` },
      { label: "이끼", svg: `<svg viewBox="0 0 64 64"><path d="M15 46 C19 33, 32 33, 36 41 C41 35, 51 38, 52 47 Z"></path><path d="M19 46 C24 41, 28 41, 32 45"></path><path d="M36 44 C40 40, 45 40, 49 44"></path><path d="M27 34 C25 28, 27 23, 32 20"></path><path d="M42 34 C43 28, 48 24, 53 24"></path></svg>` },
    ],
  },
  Musk: {
    name: "머스크",
    desc: "포근한 이불 속 같은<br>파우더리한 향",
    icons: [
      { label: "파우더리", svg: `<svg viewBox="0 0 64 64"><circle cx="22" cy="36" r="9"></circle><circle cx="36" cy="31" r="12"></circle><circle cx="45" cy="39" r="8"></circle><path d="M16 44 H50"></path><path d="M20 21 C25 17, 31 17, 36 21"></path></svg>` },
      { label: "포근함", svg: `<svg viewBox="0 0 64 64"><path d="M16 30 C16 22, 22 17, 32 17 C42 17, 48 22, 48 30 V49 H16 Z"></path><path d="M16 35 C24 40, 40 40, 48 35"></path><path d="M24 24 H40"></path><path d="M23 49 V55"></path><path d="M41 49 V55"></path></svg>` },
      { label: "머스크", svg: `<svg viewBox="0 0 64 64"><path d="M32 12 C42 24, 49 34, 49 43 C49 53, 41 58, 32 58 C23 58, 15 53, 15 43 C15 34, 22 24, 32 12Z"></path><path d="M24 44 C28 48, 36 48, 40 44"></path><path d="M25 32 C30 28, 35 28, 40 32"></path></svg>` },
    ],
  },
  Amber: {
    name: "앰버",
    desc: "따뜻하고 달콤하며<br>관능적이고 묵직한 향",
    icons: [
      { label: "앰버", svg: `<svg viewBox="0 0 64 64"><path d="M20 20 L36 14 L50 28 L43 49 L24 53 L13 37 Z"></path><path d="M20 20 L31 35 L50 28"></path><path d="M31 35 L24 53"></path><path d="M31 35 L43 49"></path></svg>` },
      { label: "바닐라", svg: `<svg viewBox="0 0 64 64"><path d="M32 13 C41 21, 46 34, 43 46 C41 55, 23 55, 21 46 C18 34, 23 21, 32 13Z"></path><path d="M32 14 C29 26, 29 41, 32 54"></path><path d="M25 31 C29 34, 35 34, 39 31"></path><path d="M24 43 C29 46, 35 46, 40 43"></path></svg>` },
      { label: "우드", svg: `<svg viewBox="0 0 64 64"><path d="M18 44 C18 30, 22 20, 32 13 C42 20, 46 30, 46 44 C46 51, 40 55, 32 55 C24 55, 18 51, 18 44Z"></path><path d="M32 13 V55"></path><path d="M24 34 C29 36, 35 36, 40 34"></path><path d="M25 45 C30 47, 36 47, 41 45"></path></svg>` },
    ],
  },
};

function buildCustomTab() {
  const wrap = document.getElementById("custom-layers");
  const labels = { top: "Top", middle: "Middle", base: "Base" };
  wrap.innerHTML = "";
  Object.keys(SCENTS).forEach((layer) => {
    const row = document.createElement("div");
    row.className = "note-row";
    row.innerHTML = `<div class="note-label">${labels[layer]}</div>`;
    SCENTS[layer].forEach((scent) => {
      const meta = SCENT_META[scent];
      const icons = meta.icons
        .map((ic) => `<span class="icon-wrap"><span class="note-icon">${ic.svg}</span><span class="icon-label">${ic.label}</span></span>`)
        .join("");
      row.innerHTML += `
        <label class="choice-card scent-${scent.toLowerCase()}">
          <input type="radio" name="custom-${layer}" data-layer="${layer}" value="${scent}" />
          <span class="icons">${icons}</span>
          <span class="choice-title">${meta.name}</span>
          <span class="choice-desc">${meta.desc}</span>
        </label>`;
    });
    wrap.appendChild(row);
  });
}

document.querySelector('[data-mode="custom"]').addEventListener("click", () => {
  const selections = { top: [], middle: [], base: [] };
  document.querySelectorAll("#custom-layers input:checked").forEach((cb) => {
    selections[cb.dataset.layer].push(cb.value);
  });
  // 레이어(Top/Middle/Base)마다 정확히 하나씩 — 라디오라 2개 이상은 불가능하고,
  // 아직 안 고른 레이어만 걸러낸다. (백엔드 _plan_from_custom도 같은 규칙 검증)
  const missing = ["top", "middle", "base"].filter((l) => selections[l].length !== 1);
  if (missing.length > 0) {
    showToast("Top, Middle, Base에서 향료를 하나씩 선택해주세요.");
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
    } else if (result.robot_failed) {
      showFailed(result.message); // 로봇이 제조 실패 신호(false)를 보낸 경우
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

// ===== 제조 실패 화면 + 10초 자동 리다이렉트 =====
function showFailed(message) {
  document.getElementById("failed-message").textContent =
    message || "로봇이 제조에 실패했습니다. 잠시 후 다시 시도해주세요.";
  showScreen("screen-failed");

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
document.getElementById("failed-home-btn").addEventListener("click", goHome);
document.getElementById("header-home").addEventListener("click", goHome);

// ===== 초기화 =====
buildCustomTab();
buildFreeTab();
updateFreeTotal();

// 제조중 화면에 시작 화면 로봇 애니메이션을 복제해 재사용 (CSS가 속도를 올림)
document
  .getElementById("making-anim")
  .appendChild(document.querySelector(".hero-anim svg").cloneNode(true));
