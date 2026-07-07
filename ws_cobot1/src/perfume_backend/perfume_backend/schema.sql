-- 조향 자동화 솔루션 DB 스키마
-- drawio 설계의 recommend_recipes 테이블 (description 컬럼은 카드 UI용 확장)
DROP TABLE IF EXISTS recommend_recipes;

CREATE TABLE recommend_recipes (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    recipe_name TEXT    NOT NULL,   -- 추천 조합명
    description TEXT,                -- 날씨/장소 등 설명 (카드 표시용)
    top_scent   TEXT    NOT NULL,   -- 탑 향료 이름 (VALID_SCENTS.top 중 하나, 예: 'Citrus')
    mid_scent   TEXT    NOT NULL,   -- 미들 향료 이름 (VALID_SCENTS.middle 중 하나)
    base_scent  TEXT    NOT NULL,   -- 베이스 향료 이름 (VALID_SCENTS.base 중 하나)
    top_ratio   INTEGER NOT NULL,   -- 탑 배율 (%)
    mid_ratio   INTEGER NOT NULL,   -- 미들 배율 (%)
    base_ratio  INTEGER NOT NULL    -- 베이스 배율 (%)
);
