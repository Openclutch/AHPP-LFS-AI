/* Automatically prestige all players. */
BEGIN TRANSACTION;

-- Increase prestige up to max 10 and reset key stats
UPDATE Players
SET
    Prestige = CASE WHEN Prestige < 10 THEN Prestige + 1 ELSE Prestige END,
    [Level] = 1,
    RollingTotalMoney = 0,
    Money = 15000,
    CampaignProgress = 0,
    TougeBattles = 0,
    PracticeBattles = 0,
    AttackBattles = 0,
    SpeedMissions = 0,
    DriftMissions = 0,
    TimeTrialMissions = 0,
    TycoonMissions = 0,
    CampaignPath = 0,
    PayBackForCar = 0,
    CopMissions = 0,
    DragBattles = 0,
    LastUpdated = GETDATE();

-- Remove cars owned by players
DELETE FROM OwnedCars;

-- Reset mission activity history
DELETE FROM MissionActivity;

COMMIT TRANSACTION;
