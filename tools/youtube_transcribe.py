"""
YouTube → Gemini Transcript Extractor
Extracts full transcripts from YouTube videos using Gemini API's native YouTube support.
"""

import os
import sys
import time
import google.generativeai as genai
from pathlib import Path
from dotenv import load_dotenv

# Load API key
for env_path in [
    Path.home() / "deepdata" / ".env",
    Path.home() / "job-search-pipeline" / ".env",
    Path.home() / "reddit_analysis_v2" / ".env",
]:
    if env_path.exists():
        load_dotenv(env_path)
        break

api_key = os.getenv("GEMINI_API_KEY")
if not api_key:
    print("ERROR: GEMINI_API_KEY not found")
    sys.exit(1)

genai.configure(api_key=api_key)

# All LEAP 71 / Lin Kayser / Josefine Lissner videos
VIDEOS = [
    {
        "id": "69OAhVh6POY",
        "title": "TEDxTUM - Let's build machines as complex as nature (Lin Kayser)",
        "duration": "14:05",
        "priority": 1,
    },
    {
        "id": "jXSif-_54jo",
        "title": "Homegrown DXB - Josefine Lissner LEAP 71",
        "duration": "46:21",
        "priority": 1,
    },
    {
        "id": "xNGKTzCJ23E",
        "title": "Computational Engineering - Josefine Lissner - Jousef Murad Podcast #114",
        "duration": "38:07",
        "priority": 1,
    },
    {
        "id": "7nA-q1wFW4Y",
        "title": "F.O.S. Josefine Lissner Interview",
        "duration": "47:43",
        "priority": 2,
    },
    {
        "id": "X0jI2cH6ADU",
        "title": "Rethinking AM with AI - Lin Kayser - S4E17",
        "duration": "31:59",
        "priority": 1,
    },
    {
        "id": "dVdG_jio-EU",
        "title": "The Engineering Singularity - The New Engineering Paradigm",
        "duration": "9:21",
        "priority": 2,
    },
    {
        "id": "Hf4NK8rPeos",
        "title": "The Next Era of Rockets - Meet the Noyron from LEAP 71",
        "duration": "12:57",
        "priority": 2,
    },
    {
        "id": "1ZJBrQGtLe4",
        "title": "We just test fired a 20000 horsepower AI-generated rocket engine",
        "duration": "0:47",
        "priority": 3,
    },
]

PROMPT = """You are a technical transcription assistant. Your task:

1. TRANSCRIBE the entire video word-for-word in the original language (English).
2. After the transcript, add a section "## KEY TECHNICAL INSIGHTS" with bullet points of:
   - Any specific engineering methods, algorithms, or approaches mentioned
   - Any details about cooling channels, heat transfer, voxels, implicits, or computational geometry
   - Any quotes about design philosophy or methodology
   - Any specific numbers, formulas, or technical specifications
   - Any mentions of PicoGK, ShapeKernel, Noyron, or other tools

Be thorough. Every technical detail matters. This is for reverse-engineering their methodology."""


def transcribe_video(video_info: dict, output_dir: Path) -> bool:
    """Transcribe a single YouTube video using Gemini API."""
    video_id = video_info["id"]
    title = video_info["title"]
    safe_title = "".join(c if c.isalnum() or c in " -_" else "" for c in title)[:80]
    output_file = output_dir / f"{safe_title}.md"

    if output_file.exists():
        print(f"  SKIP (already exists): {output_file.name}")
        return True

    url = f"https://www.youtube.com/watch?v={video_id}"
    print(f"  Processing: {title} ({video_info['duration']})")
    print(f"  URL: {url}")

    try:
        model = genai.GenerativeModel("gemini-2.0-flash")

        response = model.generate_content(
            [
                {
                    "file_data": {
                        "file_uri": url,
                        "mime_type": "video/mp4",
                    }
                },
                PROMPT,
            ],
            generation_config=genai.GenerationConfig(
                max_output_tokens=65536,
                temperature=0.1,
            ),
        )

        transcript = response.text

        # Save with metadata header
        with open(output_file, "w", encoding="utf-8") as f:
            f.write(f"# {title}\n\n")
            f.write(f"- **YouTube:** https://www.youtube.com/watch?v={video_id}\n")
            f.write(f"- **Duration:** {video_info['duration']}\n")
            f.write(f"- **Transcribed:** {time.strftime('%Y-%m-%d %H:%M')}\n\n")
            f.write("---\n\n")
            f.write(transcript)

        print(f"  SAVED: {output_file.name} ({len(transcript)} chars)")
        return True

    except Exception as e:
        print(f"  ERROR: {e}")
        # Save error for debugging
        err_file = output_dir / f"ERROR_{safe_title}.txt"
        with open(err_file, "w") as f:
            f.write(f"Video: {url}\nError: {str(e)}\n")
        return False


def main():
    output_dir = Path(__file__).parent.parent / "Knowledge" / "Transcripts"
    output_dir.mkdir(parents=True, exist_ok=True)

    # Filter by priority if arg given
    max_priority = int(sys.argv[1]) if len(sys.argv) > 1 else 3
    videos = [v for v in VIDEOS if v["priority"] <= max_priority]

    print(f"=== YouTube -> Gemini Transcript Extractor ===")
    print(f"Videos: {len(videos)} (priority <= {max_priority})")
    print(f"Output: {output_dir}")
    print()

    success = 0
    for i, video in enumerate(videos):
        print(f"[{i+1}/{len(videos)}]")
        if transcribe_video(video, output_dir):
            success += 1
        # Rate limit: wait between requests
        if i < len(videos) - 1:
            print("  Waiting 10s (rate limit)...")
            time.sleep(10)
        print()

    print(f"=== Done: {success}/{len(videos)} videos transcribed ===")
    print(f"Transcripts in: {output_dir}")


if __name__ == "__main__":
    main()
