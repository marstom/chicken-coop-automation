from __future__ import annotations
import uvicorn
from datetime import datetime, timedelta
from pathlib import Path

from fastapi import FastAPI, HTTPException, Query
from fastapi.responses import HTMLResponse
from sqlalchemy import DateTime, Float, Integer, String, Text, desc, select
from sqlalchemy.ext.asyncio import AsyncAttrs, AsyncSession, async_sessionmaker, create_async_engine
from sqlalchemy.orm import DeclarativeBase, Mapped, mapped_column
from models import (

    SensorAmmonia,
    SensorAltitude,
    SensorHumidity,
    SensorPressure,
    SensorTemperature,
    SessionLocal,
)


app = FastAPI(title="Chicken Coop Dashboard")


MODELS = {
    "temperature": SensorTemperature,
    "ammonia": SensorAmmonia,
    "pressure": SensorPressure,
    "humidity": SensorHumidity,
    "altitude": SensorAltitude,
}




@app.get("/", response_class=HTMLResponse)
async def dashboard() -> str:
    ... # TODO

@app.get("/elk")
async def elk():
    """
    Good for elastic format from esp
    ESP32 -> MQTT broker -> Elastic ingestion -> Elasticsearch -> Kibana
    {
  "@timestamp": "2026-04-19T12:34:56Z",
  "device": "chicken-coop-1",
  "temperature": 24.3,
  "humidity": 51.2,
  "pressure": 1008.7,
  "altitude": 122.4,
  "ammonia": 8.1
}
    """
    ...

@app.get("/api/series/{metric}")
async def series(
    metric: str,
    hours: int = Query(default=24, ge=1, le=24 * 30),
    limit: int = Query(default=500, ge=10, le=5000),
):
    """
    Returns a list of points for a given metric.
    Example usage: http://localhost:8000/api/series/temperature?hours=1&limit=10
    """
    model = MODELS.get(metric)
    if model is None:
        raise HTTPException(status_code=404, detail=f"Unknown metric: {metric}")

    since = datetime.utcnow() - timedelta(hours=hours)

    async with SessionLocal() as session:
        stmt = (
            select(model)
            .where(model.created_at >= since)
            .order_by(desc(model.created_at))
            .limit(limit)
        )
        rows = (await session.execute(stmt)).scalars().all()

    rows = list(reversed(rows))

    return {
        "metric": metric,
        "points": [
            {
                "created_at": row.created_at.isoformat(),
                "value": row.value,
                "topic": row.topic,
            }
            for row in rows
        ],
    }


if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)