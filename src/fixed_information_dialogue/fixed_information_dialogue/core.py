from __future__ import annotations
from dataclasses import dataclass
from datetime import datetime, timedelta
from difflib import SequenceMatcher
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple
import re
import unicodedata
import yaml

FOUND = "FOUND"
NO_MATCH = "NO_MATCH"
AMBIGUOUS = "AMBIGUOUS"
NOT_CONFIGURED = "NOT_CONFIGURED"

def normalize_text(text: str) -> str:
    text = unicodedata.normalize("NFKC", str(text)).lower().strip()
    text = re.sub(r"[^a-z0-9\s']", " ", text)
    text = re.sub(r"\s+", " ", text)
    return text.strip()

def _token_set(text: str) -> set[str]:
    return set(normalize_text(text).split())

def similarity(a: str, b: str) -> float:
    a_n, b_n = normalize_text(a), normalize_text(b)
    if not a_n or not b_n:
        return 0.0
    if a_n == b_n:
        return 1.0
    seq = SequenceMatcher(None, a_n, b_n).ratio()
    aa, bb = _token_set(a_n), _token_set(b_n)
    union = aa | bb
    jaccard = len(aa & bb) / len(union) if union else 0.0
    return 0.55 * jaccard + 0.45 * seq

@dataclass(frozen=True)
class MatchResult:
    status: str
    key: Optional[str] = None
    answer_text: Optional[str] = None
    confidence: float = 0.0
    source: Optional[str] = None
    candidates: Tuple[str, ...] = ()
    metadata: Optional[Dict[str, Any]] = None

class FixedInformationStore:
    def __init__(self, config_path: str | Path):
        self.config_path = Path(config_path)
        self.reload()

    def reload(self) -> None:
        with self.config_path.open("r", encoding="utf-8") as f:
            raw = yaml.safe_load(f) or {}
        matching = raw.get("matching", {}) or {}
        self.match_threshold = float(matching.get("threshold", 0.82))
        self.ambiguity_margin = float(matching.get("ambiguity_margin", 0.05))
        self.timezone_label = str(raw.get("timezone_label", "local"))
        self.entries = self._validate(raw.get("static_entries", {}) or {}, "static_entries")
        self.runtime_entries = self._validate(raw.get("runtime_entries", {}) or {}, "runtime_entries")

    @staticmethod
    def _validate(entries: Dict[str, Any], name: str) -> Dict[str, Dict[str, Any]]:
        if not isinstance(entries, dict):
            raise ValueError(f"{name} must be a mapping")
        out = {}
        for key, value in entries.items():
            if not isinstance(value, dict):
                raise ValueError(f"Entry {key!r} must be a mapping")
            aliases = value.get("aliases", [])
            if not isinstance(aliases, list) or not aliases:
                raise ValueError(f"Entry {key!r} needs aliases")
            value = dict(value)
            value["aliases"] = [str(x) for x in aliases]
            out[str(key)] = value
        return out

    def _aliases(self) -> Iterable[Tuple[str, str, Dict[str, Any], str]]:
        for key, entry in self.entries.items():
            for alias in entry["aliases"]:
                yield key, alias, entry, "STATIC_KB"
        for key, entry in self.runtime_entries.items():
            for alias in entry["aliases"]:
                yield key, alias, entry, "LOCAL_RUNTIME"

    def query(self, text: str, now: Optional[datetime] = None) -> MatchResult:
        if not normalize_text(text):
            return MatchResult(status=NO_MATCH)

        best = {}
        for key, alias, entry, source in self._aliases():
            score = similarity(text, alias)
            if key not in best or score > best[key][0]:
                best[key] = (score, entry, source)

        scored = sorted(
            [(score, key, entry, source) for key, (score, entry, source) in best.items()],
            reverse=True,
        )
        if not scored or scored[0][0] < self.match_threshold:
            return MatchResult(
                status=NO_MATCH,
                confidence=scored[0][0] if scored else 0.0,
                candidates=tuple(x[1] for x in scored[:3]),
            )

        top = scored[0]
        if len(scored) > 1 and top[0] - scored[1][0] < self.ambiguity_margin:
            return MatchResult(
                status=AMBIGUOUS,
                confidence=top[0],
                candidates=(top[1], scored[1][1]),
            )

        score, key, entry, source = top
        if source == "STATIC_KB":
            answer = str(entry.get("answer", "")).strip()
        else:
            answer = self._runtime_answer(entry.get("handler"), now or datetime.now())

        if not answer:
            return MatchResult(
                status=NOT_CONFIGURED, key=key, confidence=score, source=source
            )

        return MatchResult(
            status=FOUND,
            key=key,
            answer_text=answer,
            confidence=score,
            source=source,
            candidates=(key,),
            metadata={"timezone_label": self.timezone_label},
        )

    def _runtime_answer(self, handler: Any, now: datetime) -> Optional[str]:
        handler = str(handler or "")
        if handler == "time":
            return now.strftime("It is %I:%M %p.").replace(" 0", " ")
        if handler == "today":
            return now.strftime("Today is %A, %B %d, %Y.").replace(" 0", " ")
        if handler == "tomorrow":
            t = now + timedelta(days=1)
            return t.strftime("Tomorrow is %A, %B %d, %Y.").replace(" 0", " ")
        if handler == "day_of_week":
            return now.strftime("Today is %A.")
        if handler == "day_of_month":
            return f"Today is the {now.day}{self._suffix(now.day)}."
        return None

    @staticmethod
    def _suffix(day: int) -> str:
        if 10 <= day % 100 <= 20:
            return "th"
        return {1: "st", 2: "nd", 3: "rd"}.get(day % 10, "th")
