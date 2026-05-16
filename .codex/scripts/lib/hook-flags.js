#!/usr/bin/env node
/**
 * Shared hook enable/disable controls.
 *
 * Controls (RCC v2 — RCC_* preferred, ECC_* kept as compat alias):
 * - RCC_HOOK_PROFILE / ECC_HOOK_PROFILE = minimal|standard|strict (default: standard)
 * - RCC_DISABLED_HOOKS / ECC_DISABLED_HOOKS = comma,separated,hook,ids
 *
 * Precedence: RCC_* wins when set; otherwise ECC_* is read for backwards
 * compatibility with existing user setups.
 */

'use strict';

const VALID_PROFILES = new Set(['minimal', 'standard', 'strict']);

function normalizeId(value) {
  return String(value || '').trim().toLowerCase();
}

// Pick the RCC_* env var first, fall back to the legacy ECC_* alias.
function readEnvAlias(rccKey, eccKey, defaultValue = '') {
  const rcc = process.env[rccKey];
  if (rcc !== undefined && rcc !== '') return rcc;
  const ecc = process.env[eccKey];
  if (ecc !== undefined && ecc !== '') return ecc;
  return defaultValue;
}

function getHookProfile() {
  const raw = String(readEnvAlias('RCC_HOOK_PROFILE', 'ECC_HOOK_PROFILE', 'standard'))
    .trim().toLowerCase();
  return VALID_PROFILES.has(raw) ? raw : 'standard';
}

function getDisabledHookIds() {
  const raw = String(readEnvAlias('RCC_DISABLED_HOOKS', 'ECC_DISABLED_HOOKS', ''));
  if (!raw.trim()) return new Set();

  return new Set(
    raw
      .split(',')
      .map(v => normalizeId(v))
      .filter(Boolean)
  );
}

function parseProfiles(rawProfiles, fallback = ['standard', 'strict']) {
  if (!rawProfiles) return [...fallback];

  if (Array.isArray(rawProfiles)) {
    const parsed = rawProfiles
      .map(v => String(v || '').trim().toLowerCase())
      .filter(v => VALID_PROFILES.has(v));
    return parsed.length > 0 ? parsed : [...fallback];
  }

  const parsed = String(rawProfiles)
    .split(',')
    .map(v => v.trim().toLowerCase())
    .filter(v => VALID_PROFILES.has(v));

  return parsed.length > 0 ? parsed : [...fallback];
}

function isHookEnabled(hookId, options = {}) {
  const id = normalizeId(hookId);
  if (!id) return true;

  const disabled = getDisabledHookIds();
  if (disabled.has(id)) {
    return false;
  }

  const profile = getHookProfile();
  const allowedProfiles = parseProfiles(options.profiles);
  return allowedProfiles.includes(profile);
}

module.exports = {
  VALID_PROFILES,
  normalizeId,
  getHookProfile,
  getDisabledHookIds,
  parseProfiles,
  isHookEnabled,
};
