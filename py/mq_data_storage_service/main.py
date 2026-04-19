import asyncio

from typing import Awaitable, Callable

import aiomqtt
import typer

from models import (
    Base,
    DebugLog,
    SensorAmmonia,
    SensorAltitude,
    SensorHumidity,
    SensorPressure,
    SensorTemperature,
    UnknownMessage,
    engine,
    SessionLocal,
)
from sqlalchemy.ext.asyncio import AsyncSession, create_async_engine



cli = typer.Typer()




async def init_db() -> None:
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)


Handler = Callable[[AsyncSession, str, str], Awaitable[None]]


async def insert_debug_log(session: AsyncSession, topic: str, payload: str) -> None:
    session.add(DebugLog(topic=topic, payload=payload))


async def insert_temperature(session: AsyncSession, topic: str, payload: str) -> None:
    session.add(SensorTemperature(topic=topic, value=float(payload)))


async def insert_ammonia(session: AsyncSession, topic: str, payload: str) -> None:
    session.add(SensorAmmonia(topic=topic, value=float(payload)))


async def insert_pressure(session: AsyncSession, topic: str, payload: str) -> None:
    session.add(SensorPressure(topic=topic, value=float(payload)))


async def insert_humidity(session: AsyncSession, topic: str, payload: str) -> None:
    session.add(SensorHumidity(topic=topic, value=float(payload)))


async def insert_altitude(session: AsyncSession, topic: str, payload: str) -> None:
    session.add(SensorAltitude(topic=topic, value=float(payload)))


async def insert_unknown(session: AsyncSession, topic: str, payload: str) -> None:
    session.add(UnknownMessage(topic=topic, payload=payload))


ROUTES: list[tuple[str, Handler]] = [
    ("coop/log/", insert_debug_log),
    ("coop/bme280/temperature", insert_temperature),
    ("coop/ammonia/", insert_ammonia),
    ("coop/bme280/pressure", insert_pressure),
    ("coop/bme280/humidity", insert_humidity),
    ("coop/bme280/altitude", insert_altitude),
]


async def route_message(session: AsyncSession, topic: str, payload: str) -> None:
    for prefix, handler in ROUTES:
        if topic.startswith(prefix):
            await handler(session, topic, payload)
            return
    await insert_unknown(session, topic, payload)


@cli.command()
def receive_mqtt_async(
    topic: str = "coop/#",
    broker: str = "raspberrypi.local",
    username: str = "admin",
    password: str = "admin",
) -> None:
    async def main() -> None:
        await init_db()

        async with aiomqtt.Client(
            broker,
            username=username,
            password=password,
        ) as client:
            await client.subscribe(topic)

            async for message in client.messages:
                topic_str = str(message.topic)
                payload = message.payload.decode(errors="replace")

                print(f"[{topic_str}] {payload}")

                async with SessionLocal() as session:
                    try:
                        await route_message(session, topic_str, payload)
                        await session.commit()
                    except ValueError as exc:
                        await session.rollback()
                        print(f"Invalid numeric payload for {topic_str}: {payload} ({exc})")
                    except Exception:
                        await session.rollback()
                        raise

    asyncio.run(main())


if __name__ == "__main__":
    cli()