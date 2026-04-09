"""
Clean YouTube Transcript Extractor
Uses youtube-transcript-api for exact captions (no Gemini hallucination/repetition).
Then sends to Gemini ONLY for analysis (not transcription).
"""

import os
import sys
import time
import json
from pathlib import Path
from dotenv import load_dotenv

# Load API key for Gemini analysis step
for env_path in [
    Path.home() / "deepdata" / ".env",
    Path.home() / "job-search-pipeline" / ".env",
]:
    if env_path.exists():
        load_dotenv(env_path)
        break

VIDEOS = [
    {
        "id": "69OAhVh6POY",
        "title": "TEDxTUM - Let's build machines as complex as nature (Lin Kayser, 2018)",
        "speaker": "Lin Kayser",
        "lang": "en",
    },
    {
        "id": "jXSif-_54jo",
        "title": "Homegrown DXB - Josefine Lissner LEAP 71 (Dec 2025)",
        "speaker": "Josefine Lissner",
        "lang": "en",
    },
    {
        "id": "xNGKTzCJ23E",
        "title": "Computational Engineering - Josefine Lissner - Jousef Murad Podcast #114",
        "speaker": "Josefine Lissner",
        "lang": "en",
    },
    {
        "id": "7nA-q1wFW4Y",
        "title": "Future of Space - Josefine Lissner Interview",
        "speaker": "Josefine Lissner",
        "lang": "en",
    },
    {
        "id": "X0jI2cH6ADU",
        "title": "Rethinking AM with AI - Lin Kayser (Hyperganic, 2021)",
        "speaker": "Lin Kayser",
        "lang": "en",
    },
    {
        "id": "dVdG_jio-EU",
        "title": "The Engineering Singularity - The New Engineering Paradigm",
        "speaker": "Lin Kayser",
        "lang": "en",
    },
    {
        "id": "Hf4NK8rPeos",
        "title": "The Next Era of Rockets - Meet the Noyron from LEAP 71",
        "speaker": "LEAP 71",
        "lang": "pt",  # only Portuguese auto-captions, will translate
        "translate_to": "en",
    },
]


def fetch_transcript(video_id: str, lang: str = "en", translate_to: str = None) -> str:
    """Fetch transcript using youtube-transcript-api."""
    from youtube_transcript_api import YouTubeTranscriptApi

    ytt = YouTubeTranscriptApi()

    if translate_to:
        # Fetch in source language, then translate
        transcript = ytt.fetch(video_id, languages=[lang])
        transcript = transcript.translate(translate_to)
    else:
        transcript = ytt.fetch(video_id, languages=[lang])

    # Build clean text with timestamps every ~60 seconds
    lines = []
    last_ts = -60
    for snippet in transcript:
        ts = snippet.start
        text = snippet.text.strip()
        if not text or text == "[Music]":
            continue

        if ts - last_ts >= 60:
            minutes = int(ts // 60)
            seconds = int(ts % 60)
            lines.append(f"\n[{minutes:02d}:{seconds:02d}]")
            last_ts = ts

        lines.append(text)

    return " ".join(lines)


def analyze_with_gemini(transcript: str, speaker: str) -> str:
    """Send clean transcript to Gemini for technical analysis only."""
    api_key = os.getenv("GEMINI_API_KEY")
    if not api_key:
        return "\n\n## KEY TECHNICAL INSIGHTS\n\n(Gemini API key not found - manual analysis needed)\n"

    try:
        import google.generativeai as genai
        genai.configure(api_key=api_key)
        model = genai.GenerativeModel("gemini-2.0-flash")

        prompt = f"""Analyze this transcript from {speaker} and extract:

## KEY TECHNICAL INSIGHTS

For each insight, quote the EXACT words from the transcript, then explain relevance.
Focus on:
1. Engineering methodology (how they design, what algorithms, what approach)
2. Cooling channels, heat transfer, fluid routing
3. Voxels, implicits, computational geometry
4. PicoGK, ShapeKernel, Noyron specifics
5. Design philosophy quotes
6. Any specific numbers, formulas, materials mentioned
7. Manufacturing (LPBF, powder, overhang, cross-section adaptation)

Be thorough. Every technical detail matters for reverse-engineering their methodology.

TRANSCRIPT:
{transcript[:30000]}"""

        response = model.generate_content(
            prompt,
            generation_config=genai.GenerationConfig(
                max_output_tokens=8192,
                temperature=0.1,
            ),
        )
        return "\n\n---\n\n" + response.text
    except Exception as e:
        return f"\n\n## KEY TECHNICAL INSIGHTS\n\n(Gemini analysis failed: {e})\n"


def process_video(video: dict, output_dir: Path) -> bool:
    """Process a single video: fetch transcript + analyze."""
    safe_title = "".join(c if c.isalnum() or c in " -_" else "" for c in video["title"])[:80]
    output_file = output_dir / f"{safe_title}.md"

    if output_file.exists():
        size = output_file.stat().st_size
        if size > 5000:  # skip if already have good content
            print(f"  SKIP (exists, {size} bytes): {output_file.name}")
            return True

    print(f"  Fetching transcript: {video['title']}")
    try:
        transcript = fetch_transcript(
            video["id"],
            video.get("lang", "en"),
            video.get("translate_to"),
        )
        print(f"  Got {len(transcript)} chars of clean transcript")
    except Exception as e:
        print(f"  FETCH ERROR: {e}")
        return False

    print(f"  Analyzing with Gemini...")
    analysis = analyze_with_gemini(transcript, video["speaker"])

    # Write output
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(f"# {video['title']}\n\n")
        f.write(f"- **YouTube:** https://www.youtube.com/watch?v={video['id']}\n")
        f.write(f"- **Speaker:** {video['speaker']}\n")
        f.write(f"- **Method:** youtube-transcript-api (exact captions) + Gemini analysis\n")
        f.write(f"- **Extracted:** {time.strftime('%Y-%m-%d %H:%M')}\n\n")
        f.write("---\n\n")
        f.write("## TRANSCRIPT\n\n")
        f.write(transcript)
        f.write(analysis)

    print(f"  SAVED: {output_file.name}")
    return True


def main():
    output_dir = Path(__file__).parent.parent / "Knowledge" / "Transcripts_v2"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Filter by index if arg given
    if len(sys.argv) > 1 and sys.argv[1].isdigit():
        idx = int(sys.argv[1])
        videos = [VIDEOS[idx]] if idx < len(VIDEOS) else VIDEOS
    else:
        videos = VIDEOS

    print(f"=== Clean Transcript Extractor ===")
    print(f"Videos: {len(videos)}")
    print(f"Output: {output_dir}")
    print()

    success = 0
    for i, video in enumerate(videos):
        print(f"[{i+1}/{len(videos)}]")
        if process_video(video, output_dir):
            success += 1
        if i < len(videos) - 1:
            time.sleep(5)
        print()

    print(f"=== Done: {success}/{len(videos)} ===")


if __name__ == "__main__":
    main()
